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

export default function VehicleMarker({
  id,
  posX,
  posY,
  type,
  map,
}: VehicleMarkerProps) {
  const elRef = useRef<HTMLDivElement>(null);

  // Overlay mount/unmount
  useEffect(() => {
    if (!map || !elRef.current) return;

    const overlay = new Overlay({
      element: elRef.current,
      positioning: "center-center",
      stopEvent: false,
    });

    overlay.set("id", id);
    map.addOverlay(overlay);

    return () => {
      map.removeOverlay(overlay);
    };
  }, [id, map]);

  // 좌표 변경 시 Overlay 위치 갱신
  useEffect(() => {
    if (map) {
      const overlay = map
        .getOverlays()
        .getArray()
        .find((o) => o.get("id") === id);
      overlay?.setPosition([posX, posY]);
    }
  }, [posX, posY, id, map]);

  return (
    <div ref={elRef} className="transform -translate-x-1/2 -translate-y-1/2">
      {type === "police" ? (
        <svg width="20" height="20" viewBox="0 0 20 20">
          <circle
            cx="10"
            cy="10"
            r="9"
            stroke="#5c80fc"
            strokeWidth="1.5"
            fill="none"
          />
          <circle cx="10" cy="10" r="6.5" fill="#5c80fc" />
        </svg>
      ) : (
        <svg width="20" height="20" viewBox="0 0 20 20">
          <circle
            cx="10"
            cy="10"
            r="9"
            stroke="#da4341"
            strokeWidth="1.5"
            fill="none"
          />
          <circle cx="10" cy="10" r="6.5" fill="#da4341" />
        </svg>
      )}
    </div>
  );
}
