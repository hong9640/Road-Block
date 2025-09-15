// Business Types

export type VehicleType = "POLICE" | "RUNNER";

export type Status = "RUN" | "CATCH";

export interface Vehicle {
  id: number;
  vehicle_id: number;
  car_name: string;
  vehicle_type: VehicleType;
  created_at: string;       // ISO 8601
}

export interface carStatus extends Vehicle {
  location: [number, number];       // [x, y]
  status: "NORMAL" | "HALF_DESTROYED" | "COMPELTE_DESTROYED";
  fuel: number;
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

// Programming Type
export type SocketItem = {
  [url: string]: WebSocket;
};

export type MessageHandler = (data: ArrayBuffer) => void;
