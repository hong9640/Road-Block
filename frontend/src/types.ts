// Business Types
export type VehicleType = "police" | "runner";
export type DamageLevel = "normal" | "half_destroyed" | "complete_destroyed";
export type RunnerStatus = "run" | "catch";

export interface Vehicle {
  id: number;
  car_name: string;
  vehicle_type: VehicleType;
  details: CarDetail | null;
}

export interface CarDetail {
  colision_count: number;
  fuel: number;
  status: DamageLevel;
}

export interface CarPosition {
  id: number;
  posX: number;
  posY: number;
}

export interface Log {
  id: number;
  catcher_id: number | null;
  runner_id: number;
  status: RunnerStatus;
  created_at: string; // ISO 8601
}

export interface Map {
  id: number;
  map_name: string;
}

// Programming Type
export type SocketItem = {
  [url: string]: WebSocket;
};

export type MessageHandler = (data: ArrayBuffer) => void;
