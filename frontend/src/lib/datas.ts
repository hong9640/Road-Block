import type { CarPosition } from "@/types";

export const mapsData = [
  { id: 1, label: "K-City", imgUrl: "/image/K-City.webp" },
  { id: 2, label: "녹색 도시", imgUrl: "/image/GreenCity.webp" },
  { id: 3, label: "TechTown", imgUrl: "/image/TechTown.webp" },
  { id: 4, label: "상암 DMC", imgUrl: "/image/Sangam.webp" },
  { id: 5, label: "세종 BRT", imgUrl: "/image/SejongBRT.webp" },
  { id: 6, label: "엑스포광장", imgUrl: "/image/ExpoHall.webp" },
];

export const PosData: CarPosition[] = [
  {id: 6, map_id: 3, posX: 630, posY: -810},
  {id: 53, map_id: 3, posX: 620, posY: -800},
  {id: 59, map_id: 2, posX: 0, posY: 45},
  {id: 95, map_id: 2, posX: 125, posY: 1300},
  {id: 96, map_id: 3, posX: 100, posY: -110},
  {id: 97, map_id: 2, posX: -445, posY: 1000},
  {id: 107, map_id: 6, posX: -235, posY: 0},
]