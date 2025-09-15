import MapView from "@/features/map/Components/MapView";
import VehicleMarker from "@/features/map/Components/VehicleMarker";
import { useParams } from "react-router-dom";

export const MapPage: React.FC = () => {
  const mapId = Number(useParams().id);

  return (
    <main style={{ position: "relative", height: "100vh", width: "100%" }}>
      <MapView mapId={mapId} />
    </main>
  );
}
