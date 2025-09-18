import { useMemo, useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { Menu, X } from "lucide-react";

import { mapsData } from "@/__mocks__";
import type { DashboardContext } from "@/components/DashboardLayout";

import LogListItem from "./LogListItem";
import VehicleListItem from "./VehicleListItem";

import { useVehicleStore } from "@/stores/useVehicleStore";
import { useEventStore } from "@/stores/useEventStore";

import { getEventListAPI, getVehicleListAPI } from "@/Apis";

export default function ControlSideBar({
  isOpen,
  setIsOpen,
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

          {/* 내용 영역: 남는 높이를 채우도록 flex-1 */}
          <div className="flex-1 flex flex-col">
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

            {/* 차량 관리 (스크롤 영역) */}
            <section className="px-2 pt-2">
              <div className="flex items-center justify-between p-2 pr-3 mb-2">
                <span>차량 목록</span>

                {/* 차량 유형 선택 Select */}
                <div className="flex items-center gap-2">
                  <VehicleTypeSelect
                    typeFilter={typeFilter}
                    setTypeFilter={setTypeFilter}
                  />
                </div>
              </div>

              <VehicleList typeFilter={typeFilter} />
            </section>

            {/* 사건 기록 (하단 고정) */}
            <section className="mt-auto p-2 border-t border-gray-700">
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

        {/* 사이드바 토글 버튼 */}
        <button
          onClick={() => setIsOpen(!isOpen)}
          className="absolute top-4 right-[-52px] z-50 p-2 bg-white border rounded hover:bg-gray-100 focus:outline-none focus:ring-2 focus:ring-blue-700"
          aria-label={isOpen ? "사이드바 닫기" : "사이드바 열기"}
        >
          {isOpen ? (
            <X size={20} strokeWidth={3} />
          ) : (
            <Menu size={20} strokeWidth={3} />
          )}
        </button>
      </div>
    </div>
  );
}

/* 차량 유형 셀렉트 + 필터링된 리스트 */
function VehicleList({ typeFilter }: { typeFilter: string }) {
  const activeCars = useVehicleStore((s) => s.activeCars);

  const filtered = useMemo(() => {
    if (!typeFilter || typeFilter === "all") return activeCars;
    return activeCars.filter((c) => c.vehicle_type === typeFilter);
  }, [activeCars, typeFilter]);

  return (
    <div className="flex-1 overflow-y-auto min-h-0 pr-1 space-y-2">
      {filtered.map((car) => (
        <VehicleListItem key={car.id} car={car} />
      ))}
    </div>
  );
}

/* 차량 유형 Select */
// 차량 유형 Select
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
