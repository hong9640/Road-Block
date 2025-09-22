import { useMemo, useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";

import type { DashboardContext } from "@/components/DashboardLayout";
import LogListItem from "./LogListItem";
import VehicleListItem from "./VehicleListItem";
import { useVehicleStore } from "@/stores/useVehicleStore";
import { useEventStore } from "@/stores/useEventStore";

import { getEventListAPI, getVehicleListAPI } from "@/Apis";
import { mapsData } from "@/lib/datas";

export default function ControlSideBar({
  isOpen,
}: DashboardContext) {
  const nav = useNavigate();
  const events = useEventStore((s) => s.events);

  const getCars = useVehicleStore((s) => s.getCars);
  const getEvents = useEventStore((s) => s.getEvents);

  const [typeFilter, setTypeFilter] = useState<string>("all");

  useEffect(() => {
    const fetchCars = async () => {
      const carList = await getVehicleListAPI();
      getCars(carList);
    };
    const fetchEvents = async () => {
      const logList = await getEventListAPI();
      getEvents(logList);
    };
    fetchCars();
    fetchEvents();
  }, [getCars, getEvents]);

  return (
    <div className="flex h-screen">
      {/* Sidebar + Button 컨테이너 */}
      <div
        className={`
          fixed top-0 left-0 h-full w-100 transition-transform duration-400 ease-in-out
          ${isOpen ? "translate-x-0" : "-translate-x-full"}
        `}
      >
        {/* Sidebar 본체 */}
        <nav className="h-full bg-gray-800 text-white flex flex-col">
          {/* 헤더 */}
          <div className="p-4 font-bold border-b border-gray-700">
            로드블락 시스템
          </div>

          {/* 내용 영역 */}
          <div className="flex-1 flex flex-col min-h-0">
            {/* 지도 선택 */}
            <section className="p-2 space-y-2 border-b border-gray-700">
              <div className="block p-2 mb-2 hover:bg-gray-700 rounded">
                지도 선택
              </div>
              <div className="grid grid-cols-3 p-2 pr-4 gap-4">
                {mapsData.map((data) => (
                  <button
                    key={data.id}
                    type="button"
                    className="border p-2 hover:bg-gray-700 rounded-sm cursor-pointer"
                    onClick={() => nav(`maps/${data.id}`)}
                  >
                    {data.label}
                  </button>
                ))}
              </div>
            </section>

            {/* 차량 관리 */}
            <section className="px-2 pt-2 flex-1 min-h-0 flex flex-col">
              <div className="flex items-center justify-between px-3 py-2 mb-2 font-semibold">
                <span>차량 목록</span>
                <VehicleTypeSelect
                  typeFilter={typeFilter}
                  setTypeFilter={setTypeFilter}
                />
              </div>

              {/* 스크롤 가능 영역 */}
              <div className="flex-1 min-h-0 overflow-y-auto pr-1 space-y-2">
                <VehicleList typeFilter={typeFilter} />
              </div>
            </section>

            {/* 사건 기록 (하단 고정) */}
            <section className="p-2 border-t border-gray-700 min-h-[120px]">
              <div
                className="block p-2 mb-2 hover:bg-gray-700 rounded cursor-pointer"
                onClick={() => nav("logs")}
              >
                사건 기록
              </div>
              <div className="space-y-2">
                {events.slice(0, 3).map((log) => (
                  <LogListItem key={log.id} log={log} />
                ))}
              </div>
            </section>
          </div>
        </nav>
      </div>
    </div>
  );
}

/* 차량 목록 */
function VehicleList({ typeFilter }: { typeFilter: string }) {
  const activeCars = useVehicleStore((s) => s.activeCars);

  const filtered = useMemo(() => {
    if (!typeFilter || typeFilter === "all") return activeCars;
    return activeCars.filter((c) => c.vehicle_type === typeFilter);
  }, [activeCars, typeFilter]);

  return (
    <>
      {filtered.map((car) => (
        <VehicleListItem key={car.id} car={car} />
      ))}
    </>
  );
}

/* 차량 유형 Select */
function VehicleTypeSelect({
  typeFilter,
  setTypeFilter,
}: {
  typeFilter: string;
  setTypeFilter: (v: string) => void;
}) {
  const activeCars = useVehicleStore((s) => s.activeCars);

  const types = useMemo(() => {
    const set = new Set<string>(
      activeCars.map((c) => c.vehicle_type).filter(Boolean)
    );
    return ["all", ...Array.from(set)];
  }, [activeCars]);

  const label = (t: string) => {
    switch (t) {
      case "police":
        return "경찰차";
      case "runner":
        return "도주 차량";
      case "all":
      default:
        return "전체";
    }
  };

  return (
    <select
      id="vehicleType"
      value={typeFilter}
      onChange={(e) => setTypeFilter(e.target.value)}
      className="bg-gray-700 text-white text-sm rounded px-2 py-1 border border-gray-600"
    >
      {types.map((t) => (
        <option key={t} value={t}>
          {label(t)}
        </option>
      ))}
    </select>
  );
}
