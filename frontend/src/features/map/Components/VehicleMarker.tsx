import { useEffect, useRef } from "react";
import Overlay from "ol/Overlay";
import type OLMap from "ol/Map"; // OLMap 타입 명확화
import type { VehicleType } from "@/types";

interface VehicleMarkerProps {
  id: number;
  posX: number;
  posY: number;
  type: VehicleType;
  map?: OLMap | null; // 옵셔널로 변경
}

export default function VehicleMarker({
  id,
  posX,
  posY,
  type,
  map,
}: VehicleMarkerProps) {
  const elRef = useRef<HTMLDivElement | null>(null);

  // Overlay 생성 & 추가
  useEffect(() => {
    if (!map) {
      console.warn(`VehicleMarker ${id}: map이 없어 마커를 표시하지 못했습니다.`);
      return;
    }

    // React 관리 밖에서 DOM 생성
    const container = document.createElement("div");
    elRef.current = container;

    // SVG DOM 생성
    const svgNS = "http://www.w3.org/2000/svg";
    const svg = document.createElementNS(svgNS, "svg");
    svg.setAttribute("width", "20");
    svg.setAttribute("height", "20");
    svg.setAttribute("viewBox", "0 0 20 20");

    const outerCircle = document.createElementNS(svgNS, "circle");
    outerCircle.setAttribute("cx", "10");
    outerCircle.setAttribute("cy", "10");
    outerCircle.setAttribute("r", "9");
    outerCircle.setAttribute("stroke", type === "police" ? "#5c80fc" : "#da4341");
    outerCircle.setAttribute("stroke-width", "1.5");
    outerCircle.setAttribute("fill", "none");

    const innerCircle = document.createElementNS(svgNS, "circle");
    innerCircle.setAttribute("cx", "10");
    innerCircle.setAttribute("cy", "10");
    innerCircle.setAttribute("r", "6.5");
    innerCircle.setAttribute("fill", type === "police" ? "#5c80fc" : "#da4341");

    svg.appendChild(outerCircle);
    svg.appendChild(innerCircle);
    container.appendChild(svg);

    // Overlay 생성 및 추가
    const overlay = new Overlay({
      element: container,
      positioning: "center-center",
      stopEvent: false,
    });
    overlay.set("id", id);
    map.addOverlay(overlay);

    return () => {
      map.removeOverlay(overlay);
    };
  }, [id, map, type]);

  // 좌표 업데이트
  useEffect(() => {
    if (!map) return;
    const overlay = map
      .getOverlays()
      .getArray()
      .find((o) => o.get("id") === id);
    overlay?.setPosition([posX, posY]);
  }, [posX, posY, id, map]);

  return null; // React는 DOM을 직접 렌더링하지 않음
}
