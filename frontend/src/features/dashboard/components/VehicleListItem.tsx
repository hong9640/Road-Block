import type { Vehicle } from "@/types";

interface VehicleListItemProps {
  car: Vehicle;
}

export default function VehicleListItem({ car }: VehicleListItemProps) {
  return (
    <div className="p-2 mx-2 mb-2 bg-sky-100 text-black flex-1 flex justify-between">
      <span>{car.car_name}</span>
      <span>연료 {car.details?.fuel}%</span>
    </div>
  );
}
