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
  const focusedCarId = useVehicleStore((s) => s.focusedCarId);

  const [mapReady, setMapReady] = useState(false);
  const [mapError, setMapError] = useState<string | null>(null);

  const posById = useMemo<Record<number, CarPosition>>(() => {
    const acc: Record<number, CarPosition> = {};
    for (const p of carsPosition) acc[p.id] = p;
    return acc;
  }, [carsPosition]);

  useEffect(() => {
    if (!mapEl.current) return;

    setMapReady(false);
    setMapError(null);

    // 1) 로컬 XY 투영
    const code = "SIM:LOCAL";
    let localProj = getProj(code) as Projection | null;
    if (!localProj) {
      localProj = new Projection({ code, units: "m" });
    }

    // 2) 초기 View
    const tempView = new View({
      projection: localProj,
      center: [0, 0],
      zoom: 2,
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
        new MouseWheelZoom({ duration: 400 }),
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
        if (!json) {
          setMapError("지도 데이터를 불러올 수 없습니다.");
          return;
        }

        let features = [];
        try {
          features = fmt.readFeatures(json, {
            dataProjection: localProj!,
            featureProjection: localProj!,
          });
        } catch (e) {
          console.error("GeoJSON 파싱 오류:", e);
          setMapError("지도 데이터 파싱에 실패했습니다.");
          return;
        }

        if (features.length === 0) {
          setMapError("지도 데이터가 비어 있습니다.");
          return;
        }

        const source = new VectorSource({ features });
        const layer = new VectorLayer({ source, style: styleFn });
        map.addLayer(layer);

        // 6) 데이터 extent 계산
        const dataExtent = createEmpty();
        features.forEach((f) => {
          const geom = f.getGeometry();
          if (geom) extentExtend(dataExtent, geom.getExtent());
        });

        if (extentIsEmpty(dataExtent)) {
          setMapError("지도 영역이 비어 있습니다.");
          return;
        }

        // 7) 16:10을 상한으로 하되, 뷰포트 비율보다 넓히지 않음 → 과도한 여백 방지
        const dx = dataExtent[2] - dataExtent[0];
        const dy = dataExtent[3] - dataExtent[1];
        const dataRatio = dx / dy;

        const size = map.getSize();
        const viewRatio = size && size[1] > 0 ? size[0] / size[1] : 16 / 10;
        const targetRatio = Math.min(16 / 10, viewRatio);

        const finalExtent = [...dataExtent] as [number, number, number, number];

        if (dataRatio > targetRatio) {
          // 데이터가 더 넓음 → 세로 확장
          const newHeight = dx / targetRatio;
          const extra = (newHeight - dy) / 2;
          finalExtent[1] -= extra;
          finalExtent[3] += extra;
        } else {
          // 데이터가 더 높음 → 가로 확장
          const newWidth = dy * targetRatio;
          const extra = (newWidth - dx) / 2;
          finalExtent[0] -= extra;
          finalExtent[2] += extra;
        }

        // 8) 1차 fit
        const preView = new View({
          projection: localProj!,
          minZoom: 2,
          maxZoom: 22,
        });
        map.setView(preView);

        const PAD = 12; // ✅ 여백 축소
        preView.fit(finalExtent, {
          size,
          padding: [PAD, PAD, PAD, PAD],
          duration: 0,
          maxZoom: 22,
        });

        // 9) fit 해상도를 zoom-out 한계로 잠금 (OL 10.x → 새 View 교체)
        const fittedRes = preView.getResolution();
        const fittedCenter = preView.getCenter();

        // 🚩 extent를 여유 있게 확장
        const EXT_PAD = 50; // 지도 이동 가능 여백(px 단위가 아니라 좌표 단위)
        const paddedExtent: [number, number, number, number] = [
          finalExtent[0] - EXT_PAD,
          finalExtent[1] - EXT_PAD,
          finalExtent[2] + EXT_PAD,
          finalExtent[3] + EXT_PAD,
        ];

        if (fittedRes && fittedCenter) {
          const lockedView = new View({
            projection: localProj!,
            center: fittedCenter,
            resolution: fittedRes,
            maxResolution: fittedRes, // 축소 한계
            minZoom: 2,
            maxZoom: 22,
            extent: paddedExtent, // 🚩 살짝 여유 있는 extent로 제한
          });
          map.setView(lockedView);
        }

        setMapReady(true);
      })
      .catch((err) => {
        console.error("GeoJSON 로드 실패:", err);
        setMapError("네트워크 오류로 지도를 불러올 수 없습니다.");
      });

    // cleanup
    return () => {
      aborted = true;
      map.setTarget(undefined);
      mapRef.current = null;
    };
  }, [mapId]);

  // 위치 보기 클릭 시, 선택된 차량 추적
  useEffect(() => {
    if (!mapReady || !mapRef.current || !focusedCarId) return;
    const map = mapRef.current;
    const view = map.getView();

    const pos = posById[focusedCarId];
    if (!pos) return;

    view.animate({
      center: [pos.posX, pos.posY],
      duration: 500,
    });
  }, [focusedCarId, posById, mapReady]);

  // 차량 좌표 업데이트 시에도 따라가기
  useEffect(() => {
    if (!mapReady || !mapRef.current || !focusedCarId) return;
    const pos = posById[focusedCarId];
    if (!pos) return;

    mapRef.current?.getView().setCenter([pos.posX, pos.posY]);
  }, [focusedCarId, posById, mapReady]);

  return (
    <div ref={mapEl} className="w-full h-full relative bg-gray-300">
      {mapError && (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-200 font-semibold">
          {mapError}
        </div>
      )}

      {mapReady &&
        mapRef.current &&
        (activeCars ?? [])
          .map((v) => ({ v, pos: posById[v.id] }))
          .filter(({ pos }) => pos?.map_id === mapId)
          .map(({ v, pos }) => (
            <VehicleMarker
              key={v.id}
              id={v.id}
              posX={pos!.posX}
              posY={pos!.posY}
              type={v.vehicle_type}
              map={mapRef.current ?? undefined}
            />
          ))}
    </div>
  );
}
