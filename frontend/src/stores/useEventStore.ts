import type { Log } from "@/types";
import { create } from "zustand";

interface EventState {
  events: Log[];

  getEvents: (list: Log[]) => void;
  addEvents: (
    runner_id: number,
    catcher_id: number | null,
    status: Log["status"]
  ) => void;
}

export const useEventStore = create<EventState>()((set) => ({
  events: [],

  getEvents: (list) => set(() => ({ events: list })),
  addEvents: (runner_id, catcher_id, status) =>
    set((state) => {
      // 현재 저장된 이벤트 중 가장 큰 id를 찾고 +1
      const maxId =
        state.events.length > 0
          ? Math.max(...state.events.map((e) => e.id))
          : 0;

      const newEvent: Log = {
        id: maxId + 1,
        runner_id,
        catcher_id,
        status,
        created_at: new Date().toISOString(),
      };

      return { events: [newEvent, ...state.events] };
    }),
}));
