import type { Log, RunnerStatus } from "@/types";
import { formatDateMMDD, formatTimeHHMMSS } from "@/utils/timeformat";
import clsx from "clsx";

interface LogListItemProps {
  log: Log;
}

function eventBrief(event: RunnerStatus) {
  switch (event) {
    case "run":
      return {
        style: "bg-yellow-100",
        eventWord: "도주차 추적 시작",
      };
    case "catch":
      return {
        style: "bg-green-100",
        eventWord: "도주차 검거",
      };
  }
}

export default function LogListItem({ log }: LogListItemProps) {
  return (
    <div className="flex flex-cols px-2 mb-2 gap-2 text-black">
      <div className={clsx(eventBrief(log.status).style, "p-2")}>
        {formatDateMMDD(log.created_at)}
      </div>
      <div className={clsx(eventBrief(log.status).style, "p-2")}>
        {formatTimeHHMMSS(log.created_at)}
      </div>
      <div className={clsx(eventBrief(log.status).style, "p-2 flex-1")}>
        {eventBrief(log.status).eventWord}
      </div>
    </div>
  );
}
