import React from "react";
import clsx from "clsx";

interface MapCardProps {
  label: string;
  imgUrl: string;
  href?: string;              // 카드 전체를 링크로 쓰고 싶을 때
  onClick?: () => void;       // 버튼/카드 클릭 핸들러
  imgHeight?: number;         // 고정 높이 (px). 기본 200
}

const MapCard: React.FC<MapCardProps> = ({
  label,
  imgUrl,
  href,
  onClick,
  imgHeight = 200,
}) => {
  const Container: React.ElementType = href ? "a" : onClick ? "button" : "div";

  return (
    <Container
      href={href}
      onClick={onClick}
      className={clsx(
        // 카드 컨테이너
        "group block w-full overflow-hidden rounded-2xl border bg-card text-card-foreground shadow-sm transition-shadow",
        "hover:shadow-md focus:outline-none focus-visible:ring-2 focus-visible:ring-ring/60",
        // 버튼 기본 스타일 제거
        "disabled:opacity-50 disabled:pointer-events-none",
      )}
      // 버튼일 때 접근성 보완
      {...(onClick && !href ? { type: "button" } : {})}
    >
      {/* 이미지 영역 */}
      <div
        className="relative w-full overflow-hidden"
        style={{ height: imgHeight }}
      >
        <img
          src={imgUrl}
          alt={label}
          loading="lazy"
          decoding="async"
          className={clsx(
            "h-full w-full object-cover",
            "transition-transform duration-300 ease-out group-hover:scale-[1.02]",
            // 하단 페이드 마스크 (Tailwind 임의 속성)
            "[mask-image:linear-gradient(to_bottom,rgba(0,0,0,1)_70%,rgba(0,0,0,0)_100%)]"
          )}
        />
        {/* 상단 라운드 유지 */}
        <div className="pointer-events-none absolute inset-0 rounded-t-2xl ring-1 ring-black/0" />
      </div>

      {/* 푸터(라벨) */}
      <div className="px-4 pt-2 pb-4 text-sm font-medium">{label}</div>
    </Container>
  );
};

export default MapCard;
