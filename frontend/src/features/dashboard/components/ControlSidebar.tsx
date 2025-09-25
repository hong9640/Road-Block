import { useMemo, useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";

import type { DashboardContext } from "@/components/DashboardLayout";
import LogListItem from "./LogListItem";
import VehicleListItem from "./VehicleListItem";
import { useVehicleStore } from "@/stores/useVehicleStore";
import { useEventStore } from "@/stores/useEventStore";

import { getEventListAPI, getVehicleListAPI } from "@/Apis";
import { mapsData } from "@/lib/datas";

import "./ControlSidebar.css";

export default function ControlSideBar({ isOpen }: DashboardContext) {
  const nav = useNavigate();
  const events = useEventStore((s) => s.events);

  const getCars = useVehicleStore((s) => s.getCars);
  const getEvents = useEventStore((s) => s.getEvents);

  const [typeFilter, setTypeFilter] = useState<string>("all");

  useEffect(() => {
    const fetchCars = async () => {
      try {
        const carList = await getVehicleListAPI();
        getCars(carList);
      } catch (e) {
        console.error("차량 목록 불러오기 실패:", e);
      }
    };
    const fetchEvents = async () => {
      try {
        const logList = await getEventListAPI();
        getEvents(logList);
      } catch (e) {
        console.error("사건 기록 불러오기 실패:", e);
      }
    };
    fetchCars();
    fetchEvents();
  }, [getCars, getEvents]);

  return (
    <div className="flex h-screen">
      <div
        className={`sidebar-container ${
          isOpen ? "sidebar-open" : "sidebar-closed"
        }`}
      >
        <nav className="sidebar">
          {/* 헤더 */}
          <div className="sidebar-header">로드블락 시스템</div>

          <div className="flex-1 flex flex-col min-h-0">
            {/* 지도 선택 */}
            <section className="sidebar-section">
              <div className="sidebar-section-title">지도 선택</div>
              <div className="map-grid">
                {mapsData.map((data) => (
                  <button
                    key={data.id}
                    type="button"
                    className="map-button"
                    onClick={() => nav(`maps/${data.id}`)}
                  >
                    {data.label}
                  </button>
                ))}
              </div>
            </section>

            {/* 차량 관리 */}
            <section className="vehicle-section">
              <div className="vehicle-header">
                <span>차량 목록</span>
                <VehicleTypeSelect
                  typeFilter={typeFilter}
                  setTypeFilter={setTypeFilter}
                />
              </div>
              <div className="vehicle-list">
                <VehicleList typeFilter={typeFilter} />
              </div>
            </section>

            {/* 사건 기록 */}
            <section className="logs-section">
              <div className="logs-link" onClick={() => nav("logs")}>
                <span>사건 기록</span>
                <span className="px-1">〉</span>
              </div>
              <div className="logs-list">
                {Array.isArray(events) && events.length > 0 ? (
                  events
                    .slice(0, 3)
                    .map((log) => <LogListItem key={log.id} log={log} />)
                ) : (
                  <div className="text-gray-500 text-sm p-2">
                    사건 기록이 없습니다.
                  </div>
                )}
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
    if (!Array.isArray(activeCars)) return [];

    if (!typeFilter || typeFilter === "all") return activeCars;

    // 차량 유형 필터 (경찰, 도주차)
    if (typeFilter === "police" || typeFilter === "runner") {
      return activeCars.filter((c) => c.vehicle_type === typeFilter);
    }

    // 경찰차 세부 상태 필터
    if (typeFilter.startsWith("police:")) {
      const condition = typeFilter.split(":")[1];

      return activeCars.filter((c) => {
        if (c.vehicle_type !== "police") return false;

        switch (condition) {
          case "normal":
            return (
              c.details?.fuel !== undefined &&
              c.details.fuel > 20 &&
              c.details?.status === "normal"
            );
          case "out_of_fuel":
            return c.details?.fuel !== undefined && c.details?.fuel <= 20;
          case "damaged":
            return c.details?.status === "half_destroyed";
          case "destroyed":
            return c.details?.status === "complete_destroyed";
          default:
            return true;
        }
      });
    }

    return activeCars;
  }, [activeCars, typeFilter]);

  if (!Array.isArray(filtered) || filtered.length === 0) {
    return <div className="text-gray-500 text-sm p-2">차량이 없습니다.</div>;
  }

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
    if (!Array.isArray(activeCars)) return ["all"];

    const set = new Set<string>(
      activeCars.map((c) => c.vehicle_type).filter(Boolean)
    );
    const baseTypes = ["all", ...Array.from(set)];

    // 경찰차 세부 필터 확장
    const policeSubFilters = [
      "police:normal", // 정상
      "police:out_of_fuel", // 연료 없음
      "police:damaged", // 파손
      "police:destroyed", // 전손
    ];

    return [...baseTypes, ...policeSubFilters];
  }, [activeCars]);

  const label = (t: string) => {
    switch (t) {
      case "police":
        return "경찰차(전체)";
      case "runner":
        return "도주 차량";
      case "all":
        return "전체";

      case "police:normal":
        return "경찰차 - 정상";
      case "police:out_of_fuel":
        return "경찰차 - 연료 부족";
      case "police:damaged":
        return "경찰차 - 파손";
      case "police:destroyed":
        return "경찰차 - 전손";

      default:
        return t;
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
