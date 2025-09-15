// Mapview.tsx
import { useEffect, useRef } from "react";
import "ol/ol.css";

import Map from "ol/Map";
import View from "ol/View";
import Projection from "ol/proj/Projection";
import { get as getProj } from "ol/proj";

import VectorLayer from "ol/layer/Vector";
import VectorSource from "ol/source/Vector";
import GeoJSON from "ol/format/GeoJSON";

import Style from "ol/style/Style";
import Fill from "ol/style/Fill";
import Stroke from "ol/style/Stroke";

import { buffer as extentBuffer, createEmpty, extend as extentExtend, isEmpty as extentIsEmpty } from "ol/extent";

const SURFACES_URL = "/merged_road_surfaces.geojson"; // public/에 파일 배치

export default function Mapview() {
  const mapEl = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<Map | null>(null);

  useEffect(() => {
    if (!mapEl.current) return;

    // 1) 로컬 XY 투영(단위 m 가정)
    // - 등록 후 getProj로 동일 인스턴스 재사용(OL 내부 캐시)
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
    const map = new Map({
      target: mapEl.current!,
      view: tempView,
      layers: [],
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

    fetch(SURFACES_URL)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json();
      })
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
        const PAD = 2; // m 가정
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
          extent: limitedExtent,              // 드래그 제한
          center: centerNow,
          resolution: resNow,                 // 현재 해상도 유지
          minResolution: resNow / Math.pow(2, 2), // 확대 2단계 허용
          maxResolution: resNow,                  // 더 이상 축소 불가
          smoothExtentConstraint: false,
          constrainOnlyCenter: false,
          constrainResolution: true,
        });

        // 9) 최종 View 적용
        map.setView(finalView);
      })
      .catch((err) => {
        console.error("GeoJSON 로드 실패:", err);
        // UI alert은 지양—필요 시 상위에서 처리
      });

    // 정리(cleanup): 타겟 해제 및 이벤트/레이어 정리
    return () => {
      aborted = true;
      map.setTarget(undefined);
      mapRef.current = null;
    };
  }, []);

  return (
    <div
      ref={mapEl}
      style={{
        position: "absolute",
        inset: 0,
        background: "#d9d9d9", // 밝은 회색 배경
      }}
    />
  );
}
