import { PosData } from "@/lib/datas";
import type { CarDetail, CarPosition, Vehicle } from "@/types";
import { create } from "zustand";

interface VehicleState {
  activeCars: Vehicle[];
  carsPosition: CarPosition[];
  focusedCarId: number | null;

  getCars: (list: Vehicle[]) => void;
  addCar: (newCar: Vehicle) => void;
  updatePos: (
    targetId: number,
    map_id: number,
    posX: number,
    posY: number
  ) => void;
  updateStatus: (targetId: number, details: CarDetail) => void;
  deleteCar: (targetId: number) => void;
  setFocusedCarId: (id: number | null) => void;
}

export const useVehicleStore = create<VehicleState>()((set) => ({
  activeCars: [],
  carsPosition: [...PosData],
  focusedCarId: null,

  // 차량 리스트 조회
  getCars: (list) => set(() => ({ activeCars: list })),

  // 차량 추가
  addCar: (newCar) =>
    set((state) => {
      const exists = state.activeCars.some((e) => e.id === newCar.id);
      return exists ? state : { activeCars: [...state.activeCars, newCar] };
    }),

  // 차량 위치 정보 업데이트
  updatePos: (targetId, map_id, posX, posY) =>
    set((state) => {
      const exists = state.carsPosition.some((car) => car.id === targetId);
      return exists
        ? {
            carsPosition: state.carsPosition.map((car) =>
              car.id === targetId ? { ...car, map_id, posX, posY } : car
            ),
          }
        : {
            carsPosition: [
              ...state.carsPosition,
              { id: targetId, map_id, posX, posY }, // 기본 속성 세팅 필요
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

  // 차량 제거
  deleteCar: (targetId) =>
    set((state) => ({
      activeCars: state.activeCars.filter((car) => car.id !== targetId),
      carsPosition: state.carsPosition.filter((car) => car.id !== targetId),
    })),

  // 현재 차량 id 기준, 차량 포커스 번호 등록
  setFocusedCarId: (id) => set({ focusedCarId: id }),
}));
