import { cardsData } from "@/__mocks__";
import MapCard from "./MapCard";

const MainLanding = () => {

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
