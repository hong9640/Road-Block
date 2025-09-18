import { deleteVehicleAPI } from "@/Apis";
import { useVehicleStore } from "@/stores/useVehicleStore";
import type { Vehicle } from "@/types";

interface VehicleListItemProps {
  car: Vehicle;
}

export default function VehicleListItem({ car }: VehicleListItemProps) {
  const deleteCar = useVehicleStore((s) => s.deleteCar);

  const handleDeleteCar = async (id: number) => {
    const res = await deleteVehicleAPI(id);

    if (res && res.status == 204) {
      deleteCar(id);
    }
  };

  return (
    <details className="mx-2 mb-2 rounded border border-gray-300 bg-sky-100 text-black">
      {/* 요약부: 이름 + (원하면 유형 표기) */}
      <summary className="flex justify-between items-center px-2 py-1 cursor-pointer select-none">
        <span className="font-medium">{car.car_name}</span>
        <span className="text-xs text-gray-700">{car.vehicle_type}</span>
      </summary>

      {/* 상세부: 연료/상태/액션 */}
      <div className="px-3 pb-3 pt-1 text-sm space-y-2">
        {car.details && (
          <>
            <div className="flex justify-between">
              <span className="text-gray-600">연료</span>
              <span>{car.details?.fuel ?? "-"}%</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600">상태</span>
              <span>{car.details?.status}</span>
            </div>
          </>
        )}

        <div className="pt-2 grid grid-cols-3 gap-2">
          <button className="border border-gray-400 rounded px-2 py-1 hover:bg-gray-200">
            추적 시작
          </button>
          <button className="border border-gray-400 rounded px-2 py-1 hover:bg-gray-200">
            위치 이동
          </button>
          <button
            className="border border-gray-400 rounded px-2 py-1 hover:bg-gray-200"
            onClick={() => handleDeleteCar(car.id)}
          >
            차량 제거
          </button>
        </div>
      </div>
    </details>
  );
}
