// Mapview.tsx
import { useEffect, useMemo, useRef, useState } from "react";

import "ol/ol.css";
import { defaults as defaultControls, Zoom } from "ol/control";
import OLMap from "ol/Map";
import View from "ol/View";
import Projection from "ol/proj/Projection";
import { get as getProj } from "ol/proj";

import VectorLayer from "ol/layer/Vector";
import VectorSource from "ol/source/Vector";
import GeoJSON from "ol/format/GeoJSON";
import {
  defaults as defaultInteractions,
  MouseWheelZoom,
} from "ol/interaction";

import Style from "ol/style/Style";
import Fill from "ol/style/Fill";
import Stroke from "ol/style/Stroke";
import {
  buffer as extentBuffer,
  createEmpty,
  extend as extentExtend,
  isEmpty as extentIsEmpty,
} from "ol/extent";

import { getMapAPI } from "@/Apis";
import VehicleMarker from "./VehicleMarker";
import { useVehicleStore } from "@/stores/useVehicleStore";
import type { CarPosition } from "@/types";

interface MapviewProps {
  mapId: number;
}

export default function Mapview({ mapId }: MapviewProps) {
  const mapEl = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<OLMap | null>(null);

  const carsPosition = useVehicleStore((s) => s.carsPosition);
  const activeCars = useVehicleStore((s) => s.activeCars);

  // 🚩 맵 준비 여부 상태
  const [mapReady, setMapReady] = useState(false);

  // id → 위치 매핑
  const posById = useMemo<Record<number, CarPosition>>(() => {
    const acc: Record<number, CarPosition> = {};
    for (const p of carsPosition) acc[p.id] = p;
    return acc;
  }, [carsPosition]);

  useEffect(() => {
    if (!mapEl.current) return;

    setMapReady(false); // 맵 전환 시 초기화

    // 1) 로컬 XY 투영
    const code = "SIM:LOCAL";
    let localProj = getProj(code) as Projection | null;
    if (!localProj) {
      localProj = new Projection({ code, units: "m" });
    }

    // 2) 임시 View
    const tempView = new View({
      projection: localProj,
      center: [0, 0],
      zoom: 2,
      smoothExtentConstraint: false,
      constrainOnlyCenter: false,
      constrainResolution: true,
    });

    // 3) 맵 생성
    const map = new OLMap({
      target: mapEl.current!,
      view: tempView,
      layers: [],
      controls: defaultControls({ zoom: false }).extend([
        new Zoom({
          duration: 400,
          className: "ol-zoom !top-4 !right-4 !left-auto",
        }),
      ]),
      interactions: defaultInteractions().extend([
        new MouseWheelZoom({
          duration: 400,
        }),
      ]),
    });
    mapRef.current = map;

    // 4) 벡터 스타일
    const styleFn = (_feature: unknown, resolution?: number) =>
      new Style({
        fill: new Fill({ color: "#ffffff" }),
        stroke: new Stroke({
          color: "#000000",
          width: Math.max(0.6, 1.2 / Math.sqrt(resolution || 1)),
        }),
      });

    // 5) GeoJSON 로딩
    const fmt = new GeoJSON();
    let aborted = false;

    getMapAPI(mapId)
      .then((json) => {
        if (aborted) return;

        const features = fmt.readFeatures(json, {
          dataProjection: localProj!,
          featureProjection: localProj!,
        });

        const source = new VectorSource({ features });
        const layer = new VectorLayer({ source, style: styleFn });
        map.addLayer(layer);

        // extent 계산
        const dataExtent = createEmpty();
        features.forEach((f) => {
          const geom = f.getGeometry();
          if (geom) extentExtend(dataExtent, geom.getExtent());
        });

        if (extentIsEmpty(dataExtent)) {
          console.warn("데이터 extent가 비어 있습니다.");
          return;
        }

        const PAD = 50;
        const limitedExtent = extentBuffer(dataExtent, PAD);
        localProj!.setExtent(limitedExtent);

        // View 설정
        tempView.fit(dataExtent, { padding: [24, 24, 24, 24], duration: 0 });
        const resNow = tempView.getResolution()!;
        const centerNow = tempView.getCenter()!;

        const finalView = new View({
          projection: localProj!,
          extent: limitedExtent,
          center: centerNow,
          resolution: resNow,
          minZoom: 2,
          maxZoom: 20,
        });

        map.setView(finalView);

        // ✅ 뷰 세팅 완료 후 마커 표시 가능
        setMapReady(true);
      })
      .catch((err) => {
        console.error("GeoJSON 로드 실패:", err);
      });

    // cleanup
    return () => {
      aborted = true;
      map.setTarget(undefined);
      mapRef.current = null;
    };
  }, [mapId]);

  return (
    <div ref={mapEl} className="w-full h-full relative bg-gray-300">
      {mapReady &&
        mapRef.current &&
        activeCars
          .map((v) => ({ v, pos: posById[v.id] }))
          .filter(({ pos }) => !!pos && pos.map_id === mapId)
          .map(({ v, pos }) => (
            <VehicleMarker
              key={v.id}
              id={v.id}
              posX={pos!.posX}
              posY={pos!.posY}
              type={v.vehicle_type}
              map={mapRef.current}
            />
          ))}
    </div>
  );
}
