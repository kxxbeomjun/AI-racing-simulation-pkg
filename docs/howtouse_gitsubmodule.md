# Git Submodule 관련 repository 업데이트 방법

## 부모 패키지 clone
```bash
$ git clone $(부모패키지 URL).git
$ git submodule init
$ git submodule update
```

## 부모패키지에서 Submodule 내용 수정후, 부모패키지 (원격) 업데이트 방법

submodule의 commit & push를 우선 수행후, 부모패키지를 commit & push

```bash
$ cd $(submodule dir)
$ git add -A
$ git commit -m "Type your commit comments"
$ git push origin $(your submodule branch)

...

$ cd $(parent dir)
$ git add -A
$ git commit -m "Type your commit comments"
$ git push origin $(your parent branch)
```

## 외부에서 수정된 Submodule 내용을 부모패키지로 업데이트 방법

```bash
$ git submodule update
```

## Reference

https://data-engineer-tech.tistory.com/20