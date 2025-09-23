import MapView from "@/features/map/Components/MapView";
import { useParams } from "react-router-dom";

export const MapPage: React.FC = () => {
  const mapId = Number(useParams().id);

  return (
    <main className="page-map">
      <MapView mapId={mapId} />
    </main>
  );
}
