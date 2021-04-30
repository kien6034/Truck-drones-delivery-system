
uavs = [{'number': 3}, {'number': 4}, {'number': 1}]

uavs.sort(key=lambda x: x['number'], reverse=True)
print(uavs)