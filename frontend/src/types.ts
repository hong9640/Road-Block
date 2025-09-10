export type VehicleType = "POLICE" | "RUNNER";

export type Status = "RUN" | "CATCH";

export interface Vehicle {
  id: number;
  vehicle_id: number;
  car_name: string;
  vehicle_type: VehicleType;
  created_at: string;       // ISO 8601
}

export interface Log {
  id: number;
  catcher_id: number;
  runner_id: number;
  status: Status;
  created_at: string;       // ISO 8601
};

export interface Map {
  id: number;
  map_name: string;
}