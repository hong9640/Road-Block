import type { DashboardContext } from "@/components/DashboardLayout";
import { Menu, X } from "lucide-react";

export default function ControlSideBar({ isOpen, setIsOpen }: DashboardContext) {

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
              <a href="#" className="block p-2 hover:bg-gray-700">
                지도 선택
              </a>
            </li>
            <li>
              <a href="#" className="block p-2 hover:bg-gray-700">
                차량 관리
              </a>
            </li>
            <li>
              <a href="#" className="block p-2 hover:bg-gray-700">
                사건 기록
              </a>
            </li>
          </ul>
        </nav>

        {/* 사이드바에 붙은 버튼 */}
        <button
          onClick={() => setIsOpen(!isOpen)}
          className="absolute top-4 right-[-52px] z-50 p-2 bg-white border rounded hover:bg-gray-100 focus:outline-none focus:ring-2 focus:ring-blue-700"
          aria-label={isOpen ? "사이드바 닫기" : "사이드바 열기"}
        >
          {isOpen ? <X size={20} strokeWidth={3} /> : <Menu size={20} strokeWidth={3} />}
        </button>
      </div>
    </div>
  );
}
