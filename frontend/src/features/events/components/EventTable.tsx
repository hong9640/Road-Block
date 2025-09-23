import EventRow from "./EventRow";
import { useEventStore } from "@/stores/useEventStore";
import "./EventTable.css";

export default function EventTable() {
  const events = useEventStore((s) => s.events);

  return (
    <main>
      <h1 className="mb-6 text-3xl">사건 기록</h1>

      {/* 데스크톱/태블릿 전용 */}
      <div className="event-table-container">
        <div className="event-table-scroll">
          <table className="event-table">
            <colgroup>
              <col className="event-table-col-date" />
              <col className="event-table-col-time" />
              <col className="event-table-col-type" />
              <col />
            </colgroup>

            <thead className="event-table-head">
              <tr>
                <th scope="col" className="event-table-head-cell">
                  날짜
                </th>
                <th scope="col" className="event-table-head-cell">
                  시간
                </th>
                <th scope="col" className="event-table-head-cell">
                  분류
                </th>
                <th scope="col" className="event-table-head-cell">
                  상세
                </th>
              </tr>
            </thead>

            <tbody>
              {events.map((log) => (
                <EventRow key={log.id} log={log} />
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </main>
  );
}
