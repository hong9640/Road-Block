import clsx from "clsx";
import { Circle } from "lucide-react";

export default function VehicleMarker() {
  const markerBaseStyle = "w-6 h-6 rounded-full";
  const policeStyle = "#5c80fc";
  const runnerStyle = "#da4341";

  return (
    <svg width="32" height="32" viewBox="0 0 32 32">
      {/* 바깥 원 */}
      <circle cx="16" cy="16" r="13" stroke={policeStyle} strokeWidth="2" fill="none" />
      {/* 안쪽 원 */}
      <circle cx="16" cy="16" r="10" fill={policeStyle} />
    </svg>
  );
}
