#include "armor.h"


float getPointsDistance(const Point2f &a, const Point2f &b)
{
    float delta_x = a.x - b.x;
    float delta_y = a.y - b.y;
    //    return sqrtf(delta_x * delta_x + delta_y *delta_y);
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

void setNumScore(const int &armorNum, const int &targetNum, float &armorScore)
{
    if (targetNum == 0)
    {
        if (armorNum == 1)  armorScore += 4000; // todo chance better custom score
        else if (armorNum == 2) armorScore += 3000;
        else if (armorNum == 3) armorScore += 6000;
        else if (armorNum == 4) armorScore += 5000;
        else if (armorNum == 5) armorScore += 2000;
        else if (armorNum == 6) armorScore += 1000;
    }
    if (armorNum == targetNum)  {armorScore += 100000;}
}

bool armorCompare(const ArmorBox &a_armor, const ArmorBox &b_armor, const ArmorBox &lastArmor, const int &targetNum)
{
    float a_score = 0;
    float b_score = 0;
    a_score += a_armor.armorRect.area();
    b_score += b_armor.armorRect.area();

    setNumScore(a_armor.armorNum, targetNum, a_score);
    setNumScore(b_armor.armorNum, targetNum, b_score);

    if (lastArmor.armorNum != 0)
    {
        float a_distance = getPointsDistance(a_armor.center, lastArmor.center);
        float b_distance = getPointsDistance(b_armor.center, lastArmor.center);
        a_score -= a_distance * 2;
        b_score -= b_distance * 2;
    }
    return a_score > b_score;
}
