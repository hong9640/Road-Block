import { logsData, mapsData } from "@/__mocks__";
import type { DashboardContext } from "@/components/DashboardLayout";
import { Menu, X } from "lucide-react";
import { useNavigate } from "react-router-dom";
import LogListItem from "./LogListItem";
import VehicleListItem from "./VehicleListItem";
import { useEffect } from "react";
import { useVehicleStore } from "@/stores/useVehicleStore";
import { getVehicleListAPI } from "@/Apis";

export default function ControlSideBar({
  isOpen,
  setIsOpen,
}: DashboardContext) {
  const nav = useNavigate();
  const activeCars = useVehicleStore((s) => s.activeCars);
  const getCars = useVehicleStore((s) => s.getCars);

  useEffect(() => {
    const fetchCars = async () => {
      const list = await getVehicleListAPI();

      getCars(list);
    };

    fetchCars();
  }, []);

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
        <nav className="h-full bg-gray-800 text-white">
          <div className="p-4 font-bold border-b border-gray-700">
            로드블락 시스템
          </div>
          <ul className="p-2 space-y-2">
            <li>
              <div className="block p-2 mb-2 hover:bg-gray-700">지도 선택</div>
              <div className="grid grid-cols-3 p-2 pr-4 gap-4">
                {mapsData.map((data) => (
                  <button
                    key={data.id}
                    type="button"
                    className="border p-2 hover:bg-gray-700 rounded-sm cursor-pointer"
                    onClick={() => {
                      nav(`maps/${data.id}`);
                    }}
                  >
                    {data.label}
                  </button>
                ))}
              </div>
            </li>
            <li>
              <div className="block p-2 mb-2 hover:bg-gray-700">차량 관리</div>
              {activeCars
                .filter((car) => car.vehicle_type == "police")
                .map((car) => (
                  <VehicleListItem key={car.id} car={car} />
                ))}
            </li>
            <li>
              <div className="block">
                <a href="/logs" className="block p-2 mb-2 hover:bg-gray-700">
                  <div>사건 기록</div>
                </a>
              </div>
              {logsData.slice(0, 3).map((log) => (
                <LogListItem key={log.id} log={log} />
              ))}
            </li>
          </ul>
        </nav>

        {/* 사이드바에 붙은 버튼 */}
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
