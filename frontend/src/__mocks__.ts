import type { CarPosition, CarDetail, Log, Vehicle } from "./types";

export const mapsData = [
  { id: 1, label: "K-City", imgUrl: "/image/K-City.webp" },
  { id: 2, label: "상암 DMC", imgUrl: "/image/Sangam.webp" },
  { id: 3, label: "세종 BRT", imgUrl: "/image/SejongBRT.webp" },
  { id: 4, label: "녹색 도시", imgUrl: "/image/GreenCity.webp" },
  { id: 5, label: "TechTown", imgUrl: "/image/TechTown.webp" },
  { id: 6, label: "엑스포광장", imgUrl: "/image/ExpoHall.webp" },
];

// 최신 순 query
export const logsData: Log[] = [
  {
    id: 10,
    catcher_id: null,
    runner_id: 4,
    status: "RUN",
    created_at: "2025-09-08T10:24:49Z",
  },
  {
    id: 9,
    catcher_id: 1,
    runner_id: 3,
    status: "CATCH",
    created_at: "2025-09-07T17:49:54Z",
  },
  {
    id: 8,
    catcher_id: null,
    runner_id: 1,
    status: "RUN",
    created_at: "2025-09-07T12:07:12Z",
  },
  {
    id: 7,
    catcher_id: null,
    runner_id: 8,
    status: "RUN",
    created_at: "2025-09-07T05:40:28Z",
  },
  {
    id: 6,
    catcher_id: 1,
    runner_id: 2,
    status: "CATCH",
    created_at: "2025-09-07T04:03:55Z",
  },
  {
    id: 5,
    catcher_id: null,
    runner_id: 5,
    status: "RUN",
    created_at: "2025-09-06T23:52:49Z",
  },
  {
    id: 4,
    catcher_id: 4,
    runner_id: 2,
    status: "CATCH",
    created_at: "2025-09-06T12:40:14Z",
  },
  {
    id: 3,
    catcher_id: 4,
    runner_id: 3,
    status: "CATCH",
    created_at: "2025-09-05T16:55:33Z",
  },
  {
    id: 2,
    catcher_id: null,
    runner_id: 1,
    status: "RUN",
    created_at: "2025-09-05T10:52:21Z",
  },
  {
    id: 1,
    catcher_id: null,
    runner_id: 2,
    status: "RUN",
    created_at: "2025-09-05T08:42:59Z",
  },
];

export const carsData: Vehicle[] = [
  {
    id: 6,
    car_name: "Police1",
    vehicle_type: "police",
    details: {
      colision_count: 2,
      fuel: 85,
      status: "half_destroyed",
    },
  },
  {
    id: 10,
    car_name: "EGO_0",
    vehicle_type: "police",
    details: {
      colision_count: 0,
      fuel: 99,
      status: "normal",
    },
  },
  {
    id: 15,
    car_name: "runnerX",
    vehicle_type: "runner",
    details: null,
  },
];

export const posData: CarPosition[] = [
  { id: 6, posX: 37.1234016418457, posY: 127.56800079345703 },
  { id: 10, posX: 127.000994506836, posY: 37.50199890136719 },
];

export const statusData: { id: number; details: CarDetail }[] = [
];
