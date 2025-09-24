// Mapview.tsx
import { useEffect, useMemo, useRef } from "react";

import "ol/ol.css";
import { defaults as defaultControls, Zoom } from "ol/control";
import OLMap from "ol/Map"; // ← OpenLayers Map을 OLMap으로 alias
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

  // id → 위치 매핑 (Record로 구성하여 OL Map과의 이름 충돌 회피)
  const posById = useMemo<Record<number, CarPosition>>(() => {
    const acc: Record<number, CarPosition> = {};
    for (const p of carsPosition) acc[p.id] = p;
    return acc;
  }, [carsPosition]);

  useEffect(() => {
    if (!mapEl.current) return;

    // 1) 로컬 XY 투영(단위 m 가정)
    const code = "SIM:LOCAL";
    let localProj = getProj(code) as Projection | null;
    if (!localProj) {
      localProj = new Projection({ code, units: "m" });
    }

    // 2) 임시 View (fit 전 초기 화면)
    const tempView = new View({
      projection: localProj,
      center: [0, 0],
      zoom: 2,
      smoothExtentConstraint: false,
      constrainOnlyCenter: false,
      constrainResolution: true, // 정수 줌
    });

    // 3) 맵 생성 (레이어는 데이터 로딩 후 주입)
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
          duration: 400, // 애니메이션 부드럽게
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

        // 데이터 extent 계산
        const dataExtent = createEmpty();
        features.forEach((f) => {
          const geom = f.getGeometry();
          if (geom) extentExtend(dataExtent, geom.getExtent());
        });

        if (extentIsEmpty(dataExtent)) {
          console.warn("데이터 extent가 비어 있습니다.");
          return;
        }

        // 6) 드래그 제한용 extent (소폭 버퍼)
        const PAD = 50; // m 가정
        const limitedExtent = extentBuffer(dataExtent, PAD);

        // 투영에도 extent 설정(제약 계산 안정화)
        localProj!.setExtent(limitedExtent);

        // 7) tempView로 우선 fit → 해상도/센터 추출
        tempView.fit(dataExtent, { padding: [24, 24, 24, 24], duration: 0 });
        const resNow = tempView.getResolution()!;
        const centerNow = tempView.getCenter()!;

        // 8) 최종 View: 팬/줌 제약 설정
        const finalView = new View({
          projection: localProj!,
          extent: limitedExtent,
          center: centerNow,
          resolution: resNow,
          minZoom: 2, // 최소 줌 레벨
          maxZoom: 20, // 최대 줌 레벨 → 🔑 더 크게 확대 가능
        });

        // 9) 최종 View 적용
        map.setView(finalView);
      })
      .catch((err) => {
        console.error("GeoJSON 로드 실패:", err);
      });

    // 정리(cleanup)
    return () => {
      aborted = true;
      map.setTarget(undefined);
      mapRef.current = null;
    };
  }, [mapId]);

  return (
    <div ref={mapEl} className="w-full h-full relative bg-gray-300">
      {mapRef.current &&
        activeCars
          .map((v) => ({ v, pos: posById[v.id] }))
          .filter(({ pos }) => !!pos && pos.map_id === mapId) // 🔑 현재 mapId와 일치하는 차량만 표시
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
