import { useState, type ReactNode } from "react";
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
  const ctx: DashboardContext = { isOpen, setIsOpen };

  return (
    <div className="flex h-screen">
      {sidebar(ctx)}

      <div
        className={`flex-1 transition-all duration-400 ${
          isOpen ? "ml-100" : "ml-0"
        }`}
      >
        <Outlet context={ctx} />
      </div>
    </div>
  );
}
