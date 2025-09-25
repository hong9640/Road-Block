import { deleteVehicleAPI } from "@/Apis";
import { useVehicleStore } from "@/stores/useVehicleStore";
import type { Vehicle, VehicleType } from "@/types";
import "./VehicleListItem.css";

interface VehicleListItemProps {
  car: Vehicle;
}

export default function VehicleListItem({ car }: VehicleListItemProps) {
  const deleteCar = useVehicleStore((s) => s.deleteCar);
  const setFocusedCarId = useVehicleStore((s) => s.setFocusedCarId);
  const focusedCarId = useVehicleStore((s) => s.focusedCarId);

  const handleDeleteCar = async (id: number) => {
    const res = await deleteVehicleAPI(id);
    if (res && res.status === 204) {
      if (focusedCarId === id) {
        setFocusedCarId(null);
      }
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

  const handleTrackCar = () => {
    // 이미 선택된 차량이면 추적 해제
    if (focusedCarId === car.id) {
      setFocusedCarId(null);
    } else {
      setFocusedCarId(car.id);
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
          <button className="vehicle-action-btn" onClick={handleTrackCar}>{focusedCarId === car.id ? "위치 보기 중지" : "위치 보기"}</button>
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
