// MapView.tsx
import { useEffect, useRef } from "react";
import Map from "ol/Map.js";
import View from "ol/View.js";
import VectorLayer from "ol/layer/Vector.js";
import VectorSource from "ol/source/Vector.js";
import GeoJSON from "ol/format/GeoJSON.js";
import Style from "ol/style/Style.js";
import Stroke from "ol/style/Stroke.js";
import type { Extent } from "ol/extent";
import { getCenter as getExtentCenter } from "ol/extent.js";
import { Projection } from "ol/proj";


export default function MapView() {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const createdRef = useRef(false);

  useEffect(() => {
    if (createdRef.current || !containerRef.current) return;
    createdRef.current = true;

    // 1) 커스텀 로컬 좌표계 등록
    const localProj = new Projection({
      code: "SIM:LOCAL",
      units: "m",
      // extent: 데이터 로딩 후 설정
    });

    // 2) View (초기 값은 임시)
    const initialView = new View({
      projection: localProj,
      center: [0, 0],
      zoom: 6,
      minZoom: 2,
      maxZoom: 24,
      smoothExtentConstraint: false,
      constrainOnlyCenter: false,
      constrainResolution: true,
    });

    // 3) 해상도 기반 선폭 보정 (축소해도 테두리/안쪽 비율 유지)
    const styleFn = (_f: unknown, resolution: number) => {
      const inner = Math.min(
        6,
        Math.max(0.5, 3 - Math.log(resolution + 1e-12))
      );
      const casing = inner + 1.5;
      return [
        new Style({
          stroke: new Stroke({
            color: "#000",
            width: casing,
            lineCap: "round",
            lineJoin: "round",
          }),
        }),
        new Style({
          stroke: new Stroke({
            color: "#fff",
            width: inner,
            lineCap: "round",
            lineJoin: "round",
          }),
        }),
      ];
    };

    // 4) 벡터 레이어/소스
    const source = new VectorSource();
    const layer = new VectorLayer({ source, style: styleFn });

    // 5) 맵
    const map = new Map({
      target: containerRef.current!,
      view: initialView,
      layers: [layer],
    });

    // 6) GeoJSON 로드 (Vite base 경로 대응)
    const fmt = new GeoJSON();
    const url = `${import.meta.env.BASE_URL}merged_road_surfaces.geojson`;

    fetch(url, { headers: { Accept: "application/geo+json" } })
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        return r.json();
      })
      .then((json) => {
        // ⚠️ dataProjection/featureProjection을 명시해 투영 혼선을 방지
        const view = new View({
          projection: "EPSG:3857",
          center: [0, 0],
          zoom: 6,
        });
        const feats = fmt.readFeatures(json, {
          dataProjection: "EPSG:4326",
          featureProjection: "EPSG:3857",
        });

        source.addFeatures(feats);

        const extent = source.getExtent();
        console.log("layers:", map.getLayers().getLength());
        console.log("extent:", extent);

        if (Number.isFinite(extent[0])) {
          const padded: Extent = [
            extent[0] - 500,
            extent[1] - 500,
            extent[2] + 500,
            extent[3] + 500,
          ];

          // 7) 제약 포함한 View로 교체
          const nextView = new View({
            projection: localProj,
            center: getExtentCenter(padded),
            minZoom: 2,
            maxZoom: 24,
            extent: padded, // 드래그로 영역 이탈 방지
          });

          map.setView(nextView);
          map.getView().fit(padded, { padding: [24, 24, 24, 24], duration: 0 });
        }

        // 8) 크기 재계산 + 마지막 fit은 항상 map.getView()로
        requestAnimationFrame(() => {
          map.updateSize();
          map.getView().fit(extent, { padding: [24, 24, 24, 24], duration: 0 });
        });
      })
      .catch((e) => console.error("GeoJSON load failed:", e));

    // 9) 리사이즈 대응
    const ro = new ResizeObserver(() => map.updateSize());
    ro.observe(containerRef.current!);

    return () => {
      ro.disconnect();
      map.setTarget(undefined);
    };
  }, []);

  return (
    <div
      ref={containerRef}
      style={{ width: "100vw", height: "100vh", background: "#d9d9d9" }}
    />
  );
}
