# rokey_week5_ws
5주차 프로젝트 

#### 자기의 branch로 바꾸기
```
git checkout ~~~
```
#### commit 하고 올리는 방법 
```
git add .
```

```
git commit -m "메모"
```
```
git push origin $브랜치 이름 
```
#### main에서 pull하는 방법 

```
git pull origin main
```

#### main에서 merge하여 종합하기 

```
git checkout main 
```
```
git merge branch1
git merge branch2
git merge branch3
```

#### git에서 log 확인하여 복구 
```
git checkout main ( 브랜치)
```

```
git reset --hard origin main
```

```
git log --oneline

```

그리고 해당 번호 찾아서 

```
git checkout ~~~
```