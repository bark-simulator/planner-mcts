
typdef std::unordered_map<std::string, std::pair<double, double>> RegionDefinition;

class PriorKnowledgeRegion {
   RegionDefinition GetDefinition() const;
}