

namespace prior_knowledge {
   

typedef double KnowledgeValue;
typedef double RegionValueType;
typedef double DimensionName;
typedef std::unordered_map<DimensionName, RegionValueType> RegionValue;
typedef std::unordered_map<DimensionName, std::pair<RegionValueType, RegionValueType>> RegionBoundaries;

// to do -> do we need this actually
class KnowledgeRegion {
   KnowledgeRegion(const RegionBoundaries& region_definition)
       : region_definition_(region_definition) {}
   RegionBoundaries GetDefinition() const;

   std::vector<KnowledgeRegion> Partition(unsigned int num_partitions);

   private:
    RegionBoundaries region_definition_;
}


}