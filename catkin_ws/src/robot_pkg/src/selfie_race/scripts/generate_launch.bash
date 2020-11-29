loc=$(rospack find selfie_gazebo)
${loc}/generator/road-generator.py ${loc}/generator/presets/driving.xml | ${loc}/generator/gazebo-renderer.py -o ${loc}/worlds/generator_world  --force
mv "${loc}/worlds/generator_world/world.sdf" "${loc}/worlds/generator_world/world.world"
date=$(date +"%m-%d-%y-%H-%M-%S")
cp -r "${loc}/worlds/generator_world" "${loc}/worlds/generator_worlds_history/world_${date}"
echo "The generated world has been saved to ${loc}/worlds/generator_history/world_${date}"
roslaunch selfie_race generated.launch