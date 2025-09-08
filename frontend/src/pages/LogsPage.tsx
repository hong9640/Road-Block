import EventTable from "@/features/events/components/EventTable";
import type React from "react";

const LogsPage: React.FC = () => {
  return <div className="page p-16 w-100% h-100%">
    <EventTable />
  </div>;
};

export default LogsPage;
