import { deleteVehicleAPI } from "@/Apis";
import { useVehicleStore } from "@/stores/useVehicleStore";
import type { Vehicle, VehicleType } from "@/types";
import "./VehicleListItem.css";

interface VehicleListItemProps {
  car: Vehicle;
}

export default function VehicleListItem({ car }: VehicleListItemProps) {
  const deleteCar = useVehicleStore((s) => s.deleteCar);

  const handleDeleteCar = async (id: number) => {
    const res = await deleteVehicleAPI(id);
    if (res && res.status === 204) {
      deleteCar(id);
    }
  };

  const handleCarName = (name: VehicleType) => {
    switch (name) {
      case "police":
        return "경찰";
      case "runner":
        return "용의자";
    }
  };

  return (
    <details className="vehicle-item">
      {/* 요약부 */}
      <summary className="vehicle-summary">
        <span>{car.car_name}</span>
        <span className="vehicle-type">{handleCarName(car.vehicle_type)}</span>
      </summary>

      {/* 상세부 */}
      <div className="vehicle-details">
        {car.details && (
          <>
            <div className="vehicle-detail-row">
              <span>연료</span>
              <span>{car.details?.fuel ?? "-"}%</span>
            </div>
            <div className="vehicle-detail-row">
              <span>상태</span>
              <span>{car.details?.status}</span>
            </div>
          </>
        )}

        <div className="vehicle-actions">
          <button className="vehicle-action-btn">추적 시작</button>
          <button className="vehicle-action-btn">위치 이동</button>
          <button
            className="vehicle-action-btn"
            onClick={() => handleDeleteCar(car.id)}
          >
            차량 제거
          </button>
        </div>
      </div>
    </details>
  );
}
