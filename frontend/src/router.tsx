import { createBrowserRouter } from "react-router-dom";
import DashboardLayout from "./components/DashboardLayout";
import ControlSideBar from "./features/dashboard/components/ControlSidebar";
import MainPage from "./pages/MainPage";
import LogsPage from "./pages/LogsPage";
import { MapPage } from "./pages/MapPage";
import { WSProvider } from "./utils/WSProvider";

export const router = createBrowserRouter([
  { path: "/", element: <MainPage /> },
  {
    element: (
      <WSProvider>
        <DashboardLayout
          sidebar={({ isOpen, setIsOpen }) => (
            <ControlSideBar isOpen={isOpen} setIsOpen={setIsOpen} />
          )}
        />
      </WSProvider>
    ),
    children: [
      { path: "/logs", element: <LogsPage /> },
      { path: "/maps/:id", element: <MapPage /> },
    ],
  },
]);
