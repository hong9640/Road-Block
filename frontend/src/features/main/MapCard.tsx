import { Card, CardContent, CardFooter } from "@/components/ui/card";
import type React from "react";

interface MapCardProps {
  label: string;
  imgUrl: string;
}

const MapCard: React.FC<MapCardProps> = ({ label, imgUrl }) => {
  return (
    <Card className="p-0 gap-0">
      <CardContent className="p-0">
        <img src={imgUrl} alt={label} className="w-full h-[200px] object-cover rounded-t-xl mask-b-from-70% mask-b-to-100%" />
      </CardContent>
      <CardFooter className="mb-4 mt-2">{label}</CardFooter>
    </Card>
  );
};

export default MapCard;
