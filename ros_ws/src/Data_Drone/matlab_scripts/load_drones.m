function drones = load_drones(file_name)
    vars = whos('-file', file_name);
    drone_vars = {vars(startsWith({vars.name}, 'DroneE')).name};
    S = load(file_name, drone_vars{:});
    drones = struct2cell(S);
end