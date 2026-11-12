#pragma once
#include <vector>
#include <string>
#include <cstdint>

struct MaterialProps {
    std::string name;
    uint8_t id = 0;
    float r = 1.0f, g = 1.0f, b = 1.0f;
    float density = 1.0f;
    float bounciness = 0.2f;
};

class MaterialManager {
public:
    std::vector<MaterialProps> registry;

    void loadBaseMaterials() {
        registry.clear();

        MaterialProps ground;
        ground.name = "Ground"; ground.id = 0;
        ground.r = 0.25f; ground.g = 0.55f; ground.b = 0.28f;
        registry.push_back(ground);

        MaterialProps redBrick;
        redBrick.name = "RedBrick"; redBrick.id = 1;
        redBrick.r = 0.78f; redBrick.g = 0.24f; redBrick.b = 0.18f;
        redBrick.density = 1.8f; redBrick.bounciness = 0.15f;
        registry.push_back(redBrick);

        MaterialProps concrete;
        concrete.name = "Concrete"; concrete.id = 2;
        concrete.r = 0.62f; concrete.g = 0.64f; concrete.b = 0.66f;
        concrete.density = 2.6f; concrete.bounciness = 0.10f;
        registry.push_back(concrete);

        MaterialProps wood;
        wood.name = "WoodStick"; wood.id = 3;
        wood.r = 0.58f; wood.g = 0.38f; wood.b = 0.18f;
        wood.density = 0.75f; wood.bounciness = 0.30f;
        registry.push_back(wood);

        MaterialProps border;
        border.name = "Border"; border.id = 4;
        border.r = 0.35f; border.g = 0.35f; border.b = 0.38f;
        registry.push_back(border);

        MaterialProps smoothStone;
        smoothStone.name = "SmoothStone"; smoothStone.id = 5;
        smoothStone.r = 0.74f; smoothStone.g = 0.76f; smoothStone.b = 0.78f;
        smoothStone.density = 2.8f; smoothStone.bounciness = 0.12f;
        registry.push_back(smoothStone);
    }

    MaterialProps getMaterial(uint8_t id) {
        if (id < registry.size()) return registry[id];
        return registry[0];
    }
};

MaterialManager g_materials;
