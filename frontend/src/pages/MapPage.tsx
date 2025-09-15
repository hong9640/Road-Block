import MapView from "@/features/map/Components/MapView";
import VehicleMarker from "@/features/map/Components/VehicleMarker";

export const MapPage: React.FC = () => (
  <main style={{ position: "relative", height: "100vh", width: "100%" }}>
    <MapView />
  </main>
);