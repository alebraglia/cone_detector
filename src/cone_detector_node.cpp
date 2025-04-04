#include "cone_detector/cone_detector.hpp" // Assicurati che il percorso sia corretto

// costruttore
ConeDetectorNode::ConeDetectorNode() : Node("cone_detector")
{
  // Sottoscrizione al topic per le point cloud 2
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clusters", 10,
      std::bind(&ConeDetectorNode::filter, this, std::placeholders::_1));
  // Publisher per il risultato (vero/falso)
  publisher_ = this->create_publisher<std_msgs::msg::Bool>("cone_detected", 10);

  printf("Cone detector node started.\n");
}

void ConeDetectorNode::filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  //tempo totale funzione
  auto func_start = std::chrono::high_resolution_clock::now(); 

  // Se il numero di punti nella point cloud è inferiore a 10, esce dalla funzione
  double msg_size = msg->width * msg->height;
  if (msg_size < 10)
  {
    RCLCPP_WARN(this->get_logger(), "PointCloud troppo piccola, ignorata.");
    return;
  }
  // Conversione del messaggio ROS in point cloud PCL per la manipppolazione
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  // calcolo delle normali
  auto normals_start = std::chrono::high_resolution_clock::now(); //calcolo tempo

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;                                   // oggetto per il calcolo delle normali
  ne.setInputCloud(cloud);                                                                // imposto la point cloud per il calcolo
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); // creo un oggetto per la ricerca dei punti vicini
  ne.setSearchMethod(tree);                                                               // imposto il metodo di ricerca su ne
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());    // creo un oggetto per contenere le normali
  ne.setKSearch(60);                                                                      // Numero di vicini usati per stimare la normale
  ne.compute(*cloud_normals);                                                             // calcolo delle normali
  
  auto normals_end = std::chrono::high_resolution_clock::now();
  auto normals_duration = std::chrono::duration_cast<std::chrono::milliseconds>(normals_end - normals_start).count();
  std::cout << "Calcolo delle normali: " << normals_duration << " ms" << std::endl; // pubblica tempo
  

  // Segmentazione per il modello di cono
  auto seg_start = std::chrono::high_resolution_clock::now(); // calcolo tempo

  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; // creo un oggetto per la segmentazione basat sulle normali (ricerca del cono)
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CONE); // imposta il modello di cono
  seg.setMethodType(pcl::SAC_RANSAC);   // metodo RANSAC
  seg.setDistanceThreshold(0.1);        // soglia di distanza
  seg.setMaxIterations(1500);           // numero massimo di iterazioni
  seg.setNormalDistanceWeight(0.01);    // Peso della distanza rispetto alle normali

  // Imposta input cloud e input normals
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Segmentazione
  pcl::ModelCoefficients coefficients;                     // restituisce la posizione dell'apice, la direzione dell'asse e l'angolo di apertura
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); // indici dei punti inliers ( che appartengono al modello)
  seg.segment(*inliers, coefficients);                     // segmenta la point cloud

  auto seg_end = std::chrono::high_resolution_clock::now();
  auto seg_duration = std::chrono::duration_cast<std::chrono::milliseconds>(seg_end - seg_start).count();
  std::cout << "Segmentazione: " << seg_duration << " ms" << std::endl;


  // debug
  std::cout << "Inliers: " << inliers->indices.size() << std::endl;
  std::cout << "Punti totali: " << msg_size << std::endl;
  std::cout << "Rapporto inliners / punti totale: " << inliers->indices.size() / msg_size << std::endl;
  std::cout << "Cone Model Coefficients:" << std::endl;
  std::cout << "-------------------------" << std::endl;
  std::cout << "apex.x           : " << coefficients.values[0] << std::endl;
  std::cout << "apex.y           : " << coefficients.values[1] << std::endl;
  std::cout << "apex.z           : " << coefficients.values[2] << std::endl;
  std::cout << "axis_direction.x : " << coefficients.values[3] << std::endl;
  std::cout << "axis_direction.y : " << coefficients.values[4] << std::endl;
  std::cout << "axis_direction.z : " << coefficients.values[5] << std::endl;
  std::cout << "opening_angle    : " << coefficients.values[6] 
            << " (rad), " << coefficients.values[6] * 180.0 / M_PI << " (deg)" << std::endl;
  std::cout << "-------------------------" << std::endl;
  //

  // Determinazione se il cono è stato rilevato
  bool is_cone = true;

  if (true /* inliers->indices.size() / msg_size < 0.6 */)
  {
    // logica per i coefficienti per eliminare falsi positivi
    if (coefficients.values.size() == 7) // controllo che ci siano tutti i coefficienti
    {
      // posizione apice
      double apex_z = coefficients.values[2]; // altezza dell'apice del cono

      if (apex_z < 0.45 || apex_z > 0.75) // altezza dell'apice del cono tra 45 e 75 cm
      {
        std::cout << "Scartato: apice del cono fuori dall'intervallo atteso (Z = " << apex_z << ")." << std::endl;
        is_cone = false;
      }
      else
        std::cout << "Apice del cono a Z = " << apex_z << std::endl; // debug

      // angolo apertura cono
      double opening_angle = std::abs(coefficients.values[6]); // in radianti
      double min_angle = 2.0 * M_PI / 180.0;                   
      double max_angle = 9 * M_PI / 180.0;

      if (opening_angle < min_angle || opening_angle > max_angle)
      {
        std::cout << "Scartato: angolo non compatibile con un cono (" << opening_angle * 180.0 / M_PI << "°)." << std::endl;
        is_cone = false;
      }
      else
        std::cout << "Angolo di apertura del cono: " << opening_angle * 180.0 / M_PI << "°" << std::endl; // debug
    }
    else
      std::cout << "Errore: numero di coefficienti non valido." << std::endl;

  }
  /*
  else
  {
    std::cout << "Scartato: numero di inliers troppo basso." << std::endl;
    is_cone = false;
  }
  */

  // Pubblica il risultato come Bool
  if (is_cone == true)
    {
      // Pubblica il risultato come Bool
      std_msgs::msg::Bool result_msg;
      result_msg.data = is_cone;
      publisher_->publish(result_msg);
      std::cout << "Cone detected." << std::endl;
    }

    auto func_end = std::chrono::high_resolution_clock::now();
    auto func_duration = std::chrono::duration_cast<std::chrono::milliseconds>(func_end - func_start).count();
    std::cout << "Tempo totale funzione: " << func_duration << " ms" << std::endl; 
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
