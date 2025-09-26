-- MySQL dump 10.13  Distrib 8.0.43, for Win64 (x86_64)
--
-- Host: ssafy-mysql-db.mysql.database.azure.com    Database: s13p21a507
-- ------------------------------------------------------
-- Server version	8.0.42-azure

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `event`
--

DROP TABLE IF EXISTS `event`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `event` (
  `event_id` int NOT NULL AUTO_INCREMENT,
  `catcher_id` int DEFAULT NULL,
  `runner_id` int NOT NULL,
  `status` enum('RUN','CATCH','FAILED') COLLATE utf8mb4_bin NOT NULL,
  `created_at` datetime NOT NULL DEFAULT (now()),
  PRIMARY KEY (`event_id`),
  KEY `catcher_id` (`catcher_id`),
  KEY `runner_id` (`runner_id`),
  CONSTRAINT `event_ibfk_1` FOREIGN KEY (`catcher_id`) REFERENCES `vehicle` (`id`),
  CONSTRAINT `event_ibfk_2` FOREIGN KEY (`runner_id`) REFERENCES `vehicle` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=126 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `event`
--

LOCK TABLES `event` WRITE;
/*!40000 ALTER TABLE `event` DISABLE KEYS */;
INSERT INTO `event` VALUES (24,NULL,44,'RUN','2025-09-19 08:00:10'),(32,6,44,'CATCH','2025-09-19 08:47:06'),(36,NULL,51,'RUN','2025-09-20 06:39:24'),(37,NULL,52,'RUN','2025-09-20 07:14:09'),(38,53,52,'CATCH','2025-09-20 07:35:05'),(40,53,51,'CATCH','2025-09-20 08:10:01'),(46,NULL,60,'RUN','2025-09-22 03:53:10'),(47,59,60,'CATCH','2025-09-22 03:55:41'),(51,NULL,64,'RUN','2025-09-22 04:16:56'),(52,59,64,'CATCH','2025-09-22 04:18:00');
/*!40000 ALTER TABLE `event` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `map`
--

DROP TABLE IF EXISTS `map`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `map` (
  `map_id` int NOT NULL AUTO_INCREMENT,
  `map_name` varchar(100) COLLATE utf8mb4_bin NOT NULL,
  `description` varchar(255) COLLATE utf8mb4_bin DEFAULT NULL,
  `created_at` datetime NOT NULL DEFAULT (now()),
  PRIMARY KEY (`map_id`),
  UNIQUE KEY `ix_map_map_name` (`map_name`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `map`
--

LOCK TABLES `map` WRITE;
/*!40000 ALTER TABLE `map` DISABLE KEYS */;
/*!40000 ALTER TABLE `map` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `policecar`
--

DROP TABLE IF EXISTS `policecar`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `policecar` (
  `vehicle_id` int NOT NULL,
  `fuel` int NOT NULL,
  `collision_count` int NOT NULL,
  `status` enum('NORMAL','HALF_DESTROYED','COMPLETE_DESTROYED') COLLATE utf8mb4_bin NOT NULL,
  PRIMARY KEY (`vehicle_id`),
  CONSTRAINT `policecar_ibfk_1` FOREIGN KEY (`vehicle_id`) REFERENCES `vehicle` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `policecar`
--

LOCK TABLES `policecar` WRITE;
/*!40000 ALTER TABLE `policecar` DISABLE KEYS */;
INSERT INTO `policecar` VALUES (6,60,1,'HALF_DESTROYED'),(53,80,1,'HALF_DESTROYED'),(59,0,2,'COMPLETE_DESTROYED'),(95,0,0,'NORMAL'),(96,0,1,'HALF_DESTROYED'),(97,62,0,'NORMAL'),(107,100,0,'NORMAL');
/*!40000 ALTER TABLE `policecar` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `vehicle`
--

DROP TABLE IF EXISTS `vehicle`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `vehicle` (
  `id` int NOT NULL AUTO_INCREMENT,
  `vehicle_id` int NOT NULL,
  `car_name` varchar(50) COLLATE utf8mb4_bin NOT NULL,
  `vehicle_type` enum('POLICE','RUNNER') COLLATE utf8mb4_bin NOT NULL,
  `created_at` datetime NOT NULL DEFAULT (now()),
  `deleted_at` datetime DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `ix_vehicle_vehicle_id` (`vehicle_id`),
  UNIQUE KEY `ix_vehicle_car_name` (`car_name`)
) ENGINE=InnoDB AUTO_INCREMENT=221 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `vehicle`
--

LOCK TABLES `vehicle` WRITE;
/*!40000 ALTER TABLE `vehicle` DISABLE KEYS */;
INSERT INTO `vehicle` VALUES (6,123,'Police1','POLICE','2025-09-10 06:38:49',NULL),(44,9901,'Runner-112','RUNNER','2025-09-19 08:00:10','2025-09-19 06:58:30'),(51,9915,'Runner-015','RUNNER','2025-09-20 06:39:24',NULL),(52,9916,'Runner-016','RUNNER','2025-09-20 07:14:09',NULL),(53,9119,'Police-911','POLICE','2025-09-20 07:15:48',NULL),(59,9334,'Police-934','POLICE','2025-09-22 03:49:15',NULL),(60,1139,'Runner-113','RUNNER','2025-09-22 03:53:10','2025-09-19 06:58:30'),(64,9999,'Runner-900','RUNNER','2025-09-22 04:16:56','2025-09-19 06:58:30'),(95,3,'POLICE_2','POLICE','2025-09-22 08:16:41',NULL),(96,0,'POLICE_1','POLICE','2025-09-22 08:16:41',NULL),(97,4,'POLICE_3','POLICE','2025-09-22 08:16:41',NULL),(107,1111,'Police-111','POLICE','2025-09-23 04:27:40','2025-09-25 08:24:12');
/*!40000 ALTER TABLE `vehicle` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `vehiclelocation`
--

DROP TABLE IF EXISTS `vehiclelocation`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `vehiclelocation` (
  `location_id` int NOT NULL AUTO_INCREMENT,
  `vehicle_id` int NOT NULL,
  `position_x` float NOT NULL,
  `position_y` float NOT NULL,
  `created_at` datetime NOT NULL DEFAULT (now()),
  PRIMARY KEY (`location_id`),
  KEY `vehicle_id` (`vehicle_id`),
  CONSTRAINT `vehiclelocation_ibfk_1` FOREIGN KEY (`vehicle_id`) REFERENCES `vehicle` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=42647 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `vehiclelocation`
--

LOCK TABLES `vehiclelocation` WRITE;
/*!40000 ALTER TABLE `vehiclelocation` DISABLE KEYS */;
/*!40000 ALTER TABLE `vehiclelocation` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Dumping events for database 's13p21a507'
--

--
-- Dumping routines for database 's13p21a507'
--
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-09-26 10:12:06
