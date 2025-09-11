import type { Log } from "@/types";
import { create } from "zustand";

interface EventState {
  events: Log[];

  getEvents: (list: Log[]) => void;
  addEvents: (newEvent: Log) => void;
}

export const useEventStore = create<EventState>()((set) => ({
  events: [],

  getEvents: (list) => set(() => ({ events: list })),
  addEvents: (newEvent) =>
    set((state) => {
      const exists = state.events.some((e) => e.id === newEvent.id);
      return exists ? state : { events: [...state.events, newEvent] };
    }),
}));
