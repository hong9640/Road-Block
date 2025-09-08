import { createBrowserRouter } from "react-router-dom";
import DashboardLayout from "./components/DashboardLayout";
import ControlSideBar from "./features/dashboard/components/ControlSidebar";
import MainPage from "./pages/MainPage";
import LogsPage from "./pages/LogsPage";
import { MapPage } from "./pages/MapPage";

export const router = createBrowserRouter([
  { path: "/", element: <MainPage /> },
  {
    element: (
      <DashboardLayout
        sidebar={({ isOpen, setIsOpen }) => (
          <ControlSideBar isOpen={isOpen} setIsOpen={setIsOpen} />
        )}
      />
    ),
    children: [
      { path: "/logs", element: <LogsPage /> },
      { path: "/maps/:id", element: <MapPage /> },
    ],
  },
]);
