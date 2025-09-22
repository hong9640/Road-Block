import { Menu, X } from "lucide-react";
import { useEffect, useState, type ReactNode } from "react";
import { Outlet } from "react-router-dom";

export type DashboardContext = {
  isOpen: boolean;
  setIsOpen: (v: boolean) => void;
};

interface DashboardLayoutProps {
  sidebar: (ctx: DashboardContext) => ReactNode;
}

export default function DashboardLayout({ sidebar }: DashboardLayoutProps) {
  const [isOpen, setIsOpen] = useState(true);
  const [showButton, setShowButton] = useState(true); // 🔑 버튼 표시 여부
  const ctx: DashboardContext = { isOpen, setIsOpen };

  useEffect(() => {
    let timer: ReturnType<typeof setTimeout>;

    const handleActivity = () => {
      setShowButton(true);
      clearTimeout(timer);
      timer = setTimeout(() => setShowButton(false), 3000); // 3초 후 자동 숨김
    };

    // 사용자 입력 이벤트 등록 (PC + 모바일 터치 포함)
    window.addEventListener("mousemove", handleActivity);
    window.addEventListener("keydown", handleActivity);
    window.addEventListener("touchstart", handleActivity);

    // 초기 타이머 시작
    timer = setTimeout(() => setShowButton(false), 3000);

    return () => {
      clearTimeout(timer);
      window.removeEventListener("mousemove", handleActivity);
      window.removeEventListener("keydown", handleActivity);
      window.removeEventListener("touchstart", handleActivity);
    };
  }, []);

  return (
    <div className="flex h-screen relative">
      {/* Sidebar */}
      {sidebar(ctx)}

      {/* Main 영역 */}
      <div
        className={`flex-1 transition-all duration-400 ${
          isOpen ? "ml-100" : "ml-0"
        }`}
      >
        <Outlet context={ctx} />
      </div>

      {/* 🔑 토글 버튼 (자동 숨김 + fade-out 애니메이션) */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={`
          fixed top-4 z-50
          p-2 bg-white border rounded hover:bg-gray-100
          focus:outline-none focus:ring-2 focus:ring-blue-900
          transition-all duration-400
          ${isOpen ? "left-[416px]" : "left-[16px]"} 
          ${
            showButton
              ? "opacity-100 translate-x-0"
              : "opacity-0 -translate-x-4 pointer-events-none"
          }
        `}
        aria-label={isOpen ? "사이드바 닫기" : "사이드바 열기"}
      >
        {isOpen ? (
          <X size={20} strokeWidth={3} />
        ) : (
          <Menu size={20} strokeWidth={3} />
        )}
      </button>
    </div>
  );
}
