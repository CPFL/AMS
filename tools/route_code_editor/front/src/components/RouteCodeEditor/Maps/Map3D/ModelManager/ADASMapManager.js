export default class ADASMapManager extends THREE.Group {

    constructor() {
        super();
        this.name = 'ADASMapManager';
    }

    deleteADASMap() {
        for (let i = this.children.length - 1; i >= 0; i--) {
            this.remove(this.children[i]);
        }
    }

    addADASMap(adasmap){
        this.deleteADASMap();
        this.add(adasmap);
    }

    async loadADASMap(adasmap){

        if(adasmap !== null) {
            this.addADASMap(adasmap);
        }else{
            this.deleteADASMap();
        }

    }

}