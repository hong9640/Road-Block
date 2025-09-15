import type { carStatus } from "@/types";
import { create } from "zustand";

interface VehicleState {
  activeCars: carStatus[];

  getCars: (list: carStatus[]) => void;
  registerCar: (newCar: carStatus) => void;
  updatePos: (targetId: number, posX: number, posY: number) => void;
  
}

export const useVehicleStore = create<VehicleState>()((set) => ({
  activeCars: [],

  getCars: (list) => set(() => ({ activeCars: list })),
  registerCar: (newCar) =>
    set((state) => {
      const exists = state.activeCars.some((e) => e.id === newCar.id);
      return exists ? state : { activeCars: [...state.activeCars, newCar] };
    }),
  updatePos: (targetId, posX, posY) =>
    set((state) => ({
      activeCars: state.activeCars.map(
        (car) => car.id === targetId ? { ...car, posX, posY } : car
      ),
    })),
}));
