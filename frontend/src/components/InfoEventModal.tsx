// InfoEventModal.tsx (Info 전용)
import React, { useEffect, useMemo, useRef, type JSX } from "react";
import { createPortal } from "react-dom";
import { Info, X } from "lucide-react";

export interface InfoEventModalProps {
  open: boolean; // (필수) 표시 여부
  onClose: () => void; // (필수) 닫기 핸들러
  children?: React.ReactNode; // 본문 메시지
  title?: string; // 제목(기본: "알림")
  autoCloseMs?: number;
}

export function InfoEventModal({
  open,
  onClose,
  children,
  title = "알림",
  autoCloseMs,
}: InfoEventModalProps): JSX.Element | null {
  const overlayRef = useRef<HTMLDivElement | null>(null);
  const dialogRef = useRef<HTMLDivElement | null>(null);
  const lastActive = useRef<HTMLElement | null>(null);

  const titleId = useMemo(() => `i-em-title-${uid()}`, []);
  const descId = useMemo(() => `i-em-desc-${uid()}`, []);

  // 바디 스크롤 잠금 + 포커스 복귀
  useEffect(() => {
    if (!open) return;

    lastActive.current = document.activeElement as HTMLElement | null;
    const prevOverflow = document.body.style.overflow;
    document.body.style.overflow = "hidden";

    // 다음 프레임에서 다이얼로그 내 첫 포커서블 요소에 포커스
    const id = window.setTimeout(() => focusFirst(dialogRef.current), 0);

    return () => {
      window.clearTimeout(id);
      document.body.style.overflow = prevOverflow;
      lastActive.current?.focus?.();
    };
  }, [open]);

  // ESC 닫기
  useEffect(() => {
    if (!open) return;
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        e.preventDefault();
        onClose();
      }
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [open, onClose]);

  // 자동 닫기
  useEffect(() => {
    if (!open || !autoCloseMs) return;
    const timer = window.setTimeout(onClose, autoCloseMs);
    return () => window.clearTimeout(timer);
  }, [open, autoCloseMs, onClose]);

  // Tab 키 포커스 트랩
  const onKeyDown = (e: React.KeyboardEvent) => {
    if (e.key !== "Tab") return;
    const list = getFocusable(dialogRef.current);
    if (!list.length) return;

    const first = list[0];
    const last = list[list.length - 1];

    if (e.shiftKey && document.activeElement === first) {
      e.preventDefault();
      last.focus();
    } else if (!e.shiftKey && document.activeElement === last) {
      e.preventDefault();
      first.focus();
    }
  };

  // 백드롭 클릭 시 모달 닫기
  const onBackdrop = (e: React.MouseEvent) => {
    if (e.target === overlayRef.current) onClose();
  };

  if (!open) return null;

  const portalRoot = document.getElementById("modal-root") ?? document.body;

  return createPortal(
    <div ref={overlayRef} onMouseDown={onBackdrop} className="modal-overlay">
      <div
        ref={dialogRef}
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        aria-describedby={descId}
        onKeyDown={onKeyDown}
        className="modal-dialog"
      >
        {/* 헤더 */}
        <div className="modal-header">
          <div className="flex items-center gap-2">
            <Info size={22} className="modal-header-icon" aria-hidden="true" />
            <h2 id={titleId} className="modal-title">
              {title}
            </h2>
          </div>
          <button
            type="button"
            aria-label="닫기"
            onClick={onClose}
            className="modal-close-btn"
          >
            <X size={18} className="text-white" />
          </button>
        </div>

        {/* 본문 */}
        {children ? (
          <div className="modal-body" id={descId}>
            {children}
          </div>
        ) : null}
      </div>
    </div>,
    portalRoot
  );
}

/* ---------- 유틸 ---------- */
function uid() {
  return Math.random().toString(36).slice(2, 9);
}
function getFocusable(root: HTMLElement | null): HTMLElement[] {
  if (!root) return [];
  const selector =
    'a[href], area[href], input:not([disabled]):not([type="hidden"]), select:not([disabled]), textarea:not([disabled]), button:not([disabled]), iframe, object, embed, [contenteditable], [tabindex]:not([tabindex="-1"])';
  return Array.from(root.querySelectorAll<HTMLElement>(selector)).filter(
    (el) => !!(el.offsetWidth || el.offsetHeight || el.getClientRects().length)
  );
}
function focusFirst(root: HTMLElement | null) {
  const list = getFocusable(root);
  (list[0] ?? root)?.focus?.();
}
