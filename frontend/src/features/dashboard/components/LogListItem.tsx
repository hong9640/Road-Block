import type { Log, RunnerStatus } from "@/types";
import { formatDateMMDD, formatTimeHHMM } from "@/utils/timeformat";
import clsx from "clsx";

interface LogListItemProps {
  log: Log;
}

function eventBrief(event: RunnerStatus) {
  switch (event) {
    case "run":
      return {
        style: "bg-red-200",
        eventWord: "도주차 추적 시작",
      };
    case "catch":
      return {
        style: "bg-green-200",
        eventWord: "도주차 검거",
      };
  }
}

export default function LogListItem({ log }: LogListItemProps) {
  return (
    <div className={clsx(eventBrief(log.status).style, "flex justify-between px-3 py-2 mb-2")}>
      <span className="text-black">{eventBrief(log.status).eventWord}</span>
      <div className="flex self-center gap-2">
        <span className="text-gray-600 text-sm">
          {formatDateMMDD(log.created_at)}
        </span>
        <span className="text-gray-600 text-sm">
          {formatTimeHHMM(log.created_at)}
        </span>
      </div>
    </div>
  );
}
