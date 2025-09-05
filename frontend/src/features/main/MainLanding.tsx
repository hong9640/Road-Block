import MapCard from "./MapCard";

const MainLanding = () => {
  const cardsData = [
    { id: 1, label: "K-City", imgUrl: "/image/K-City.webp" },
    { id: 2, label: "상암 DMC", imgUrl: "/image/Sangam.webp" },
    { id: 3, label: "세종 BRT", imgUrl: "/image/SejongBRT.webp" },
    { id: 4, label: "녹색 도시", imgUrl: "/image/GreenCity.webp" },
    { id: 5, label: "TechTown", imgUrl: "/image/TechTown.webp" },
    { id: 6, label: "엑스포광장", imgUrl: "/image/ExpoHall.webp" },
  ];

  return (
    <>
      <h1 className="text-center text-3xl mb-16">경찰차 자율주행 로드블락 시스템</h1>
      <div className="grid grid-cols-3 gap-12">
        {cardsData.map((data) => (
          <MapCard key={data.id} imgUrl={data.imgUrl} label={data.label} />
        ))}
      </div>
    </>
  );
};

export default MainLanding;
