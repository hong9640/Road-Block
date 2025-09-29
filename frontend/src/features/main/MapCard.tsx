import React from "react";
import "./MapCard.css"

interface MapCardProps {
  label: string;
  imgUrl: string;
  href?: string;
  onClick?: () => void;
  imgHeight?: number;
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
      className="map-card group"
      {...(onClick && !href ? { type: "button" } : {})}
    >
      {/* 이미지 영역 */}
      <div className="map-card-img-container" style={{ height: imgHeight }}>
        <img
          src={imgUrl}
          alt={label}
          loading="eager"
          fetchPriority="high"
          decoding="async"
          className="map-card-img"
        />
        <div className="map-card-img-overlay" />
      </div>

      {/* 라벨 */}
      <div className="map-card-label">{label}</div>
    </Container>
  );
};

export default MapCard;
