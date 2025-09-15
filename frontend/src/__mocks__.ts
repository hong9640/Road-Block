import type { Log, Vehicle } from "./types";

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
    catcher_id: 1,
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
    catcher_id: 3,
    runner_id: 1,
    status: "RUN",
    created_at: "2025-09-07T12:07:12Z",
  },
  {
    id: 7,
    catcher_id: 2,
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
    catcher_id: 3,
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
    catcher_id: 2,
    runner_id: 1,
    status: "RUN",
    created_at: "2025-09-05T10:52:21Z",
  },
  {
    id: 1,
    catcher_id: 1,
    runner_id: 2,
    status: "RUN",
    created_at: "2025-09-05T08:42:59Z",
  },
];

export const carsData: Vehicle[] = [
  {
    id: 1,
    vehicle_id: 1,
    car_name: "경찰1",
    vehicle_type: "POLICE",
    created_at: "2025-09-05T08:42:59Z",
  },
  {
    id: 2,
    vehicle_id: 2,
    car_name: "경찰2",
    vehicle_type: "POLICE",
    created_at: "2025-09-06T23:52:49Z",
  },
  {
    id: 3,
    vehicle_id: 3,
    car_name: "도주자",
    vehicle_type: "RUNNER",
    created_at: "2025-09-07T05:40:28Z",
  },
  {
    id: 4,
    vehicle_id: 4,
    car_name: "ego-004",
    vehicle_type: "POLICE",
    created_at: "2025-09-08T10:24:49Z",
  },
];
