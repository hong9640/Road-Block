import type { CarDetail, CarPosition, Vehicle } from "@/types";
import { create } from "zustand";

interface VehicleState {
  activeCars: Vehicle[];
  carsPosition: CarPosition[];

  getCars: (list: Vehicle[]) => void;
  addCar: (newCar: Vehicle) => void;
  updatePos: (targetId: number, posX: number, posY: number) => void;
  updateStatus: (targetId: number, details: CarDetail) => void;
}

export const useVehicleStore = create<VehicleState>()((set) => ({
  activeCars: [],
  carsPosition: [],

  // 차량 리스트 조회
  getCars: (list) => set(() => ({ activeCars: list })),

  // 차량 추가
  addCar: (newCar) =>
    set((state) => {
      const exists = state.activeCars.some((e) => e.id === newCar.id);
      return exists ? state : { activeCars: [...state.activeCars, newCar] };
    }),

  // 차량 위치 정보 업데이트
  updatePos: (targetId, posX, posY) =>
    set((state) => {
      const exists = state.carsPosition.some((car) => car.id === targetId);
      return exists
        ? {
            carsPosition: state.carsPosition.map((car) =>
              car.id === targetId ? { ...car, posX, posY } : car
            ),
          }
        : {
            carsPosition: [
              ...state.carsPosition,
              { id: targetId, posX, posY }, // 기본 속성 세팅 필요
            ],
          };
    }),

  // 차량 상태 정보 업데이트
  updateStatus: (targetId, details) =>
    set((state) => ({
      activeCars: state.activeCars.map((car) =>
        car.id === targetId ? { ...car, details: details } : car
      ),
    })),
}));
