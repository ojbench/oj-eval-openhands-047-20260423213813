
import requests
import json
import os

token = os.environ.get("ACMOJ_TOKEN")
problem_id = 2284
api_base = "https://acm.sjtu.edu.cn/OnlineJudge/api/v1"

with open("src.hpp", "r") as f:
    code = f.read()

headers = {
    "Authorization": f"Bearer {token}",
    "Content-Type": "application/x-www-form-urlencoded",
}

data = {
    "language": "cpp",
    "code": code
}

response = requests.post(f"{api_base}/problem/{problem_id}/submit", headers=headers, data=data, proxies={"https": None, "http": None})
print(response.status_code)
print(response.text)
