export default function VehicleMarker() {
  const markerBaseStyle = "w-6 h-6 rounded-full";
  const policeStyle = "#5c80fc";
  const runnerStyle = "#da4341";

  return (
    <svg width="20" height="20" viewBox="0 0 20 20">
      {/* 바깥 원 */}
      <circle cx="10" cy="10" r="9" stroke={policeStyle} strokeWidth="1.5" fill="none" />
      {/* 안쪽 원 */}
      <circle cx="10" cy="10" r="6.5" fill={policeStyle} />
    </svg>
  );
}
