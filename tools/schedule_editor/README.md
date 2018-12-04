# Route Code Editor

### REQUIREMENTS
- Python 3.6.x
- Node 10.x


### SETUP

```
# Install Package
$ apt-get install vim git python-pip

# Install requiremtns
$ pip install django (version=2.0.8 recommend)

# Install npm package
$ cd ./front
$ npm install

# build(development)
$ npm run develop
# build(production)
$ npm run build
```

### TEST
```
$ cd ./schedule_editor/
$ cd templates
$ ln -s ../../front/static/schedule_editor schedule_editor
$ cd ../static
$ ln -s ../../front/static/schedule_editor schedule_editor
$ python manage.py migrate
$ python manage.py runserver
#Access localhost:8000/schedule_editor
```