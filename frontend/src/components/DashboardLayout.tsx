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
  const [showButton, setShowButton] = useState(true);
  const ctx: DashboardContext = { isOpen, setIsOpen };

  useEffect(() => {
    let timer: ReturnType<typeof setTimeout>;

    const handleActivity = () => {
      setShowButton(true);
      clearTimeout(timer);
      timer = setTimeout(() => setShowButton(false), 3000);
    };

    window.addEventListener("mousemove", handleActivity);
    window.addEventListener("keydown", handleActivity);
    window.addEventListener("touchstart", handleActivity);

    timer = setTimeout(() => setShowButton(false), 3000);

    return () => {
      clearTimeout(timer);
      window.removeEventListener("mousemove", handleActivity);
      window.removeEventListener("keydown", handleActivity);
      window.removeEventListener("touchstart", handleActivity);
    };
  }, []);

  return (
    <div className="dashboard">
      {/* Sidebar */}
      {sidebar(ctx)}

      {/* Main */}
      <div className={`dashboard-main ${isOpen ? "ml-100" : "ml-0"}`}>
        <Outlet context={ctx} />
      </div>

      {/* Toggle button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={[
          "sidebar-toggle",
          isOpen ? "sidebar-toggle-open" : "sidebar-toggle-closed",
          showButton ? "sidebar-toggle-visible" : "sidebar-toggle-hidden",
        ].join(" ")}
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
