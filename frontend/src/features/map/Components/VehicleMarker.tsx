import { useEffect, useRef } from "react";
import Overlay from "ol/Overlay";
import { Map } from "ol";
import type { VehicleType } from "@/types";

interface VehicleMarkerProps {
  id: number;
  posX: number;
  posY: number;
  type: VehicleType;
  map: Map | null;
}

export default function VehicleMarker({ id, posX, posY, type, map }: VehicleMarkerProps) {
  const elRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (!map) return;

    // React 관리 밖에서 DOM 생성
    const container = document.createElement("div");
    elRef.current = container;

    // SVG DOM 생성 (보안 친화적으로 createElementNS 사용)
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

    // OpenLayers Overlay 생성 및 추가
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

  useEffect(() => {
    if (map) {
      const overlay = map
        .getOverlays()
        .getArray()
        .find((o) => o.get("id") === id);
      overlay?.setPosition([posX, posY]);
    }
  }, [posX, posY, id, map]);

  // React는 DOM을 렌더링하지 않음
  return null;
}
