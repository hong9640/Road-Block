// Jenkinsfile

pipeline {
    agent any

    environment {
        // GitLab 프로젝트 경로를 소문자로 변환 (예: s10-final/s10p31a507)
        IMAGE_PATH = "${env.GIT_URL.replace('https://lab.ssafy.com/', '').replace('.git', '').toLowerCase()}"
        REGISTRY = "lab.ssafy.com"
        // Jenkins의 빌드 번호를 태그로 사용하여 이미지를 구분
        IMAGE_NAME = "${REGISTRY}/${IMAGE_PATH}:${env.BUILD_NUMBER}"
    }

    stages {
        stage('Build') {
            steps {
                script {
                    echo "Checking out from branch: ${env.BRANCH_NAME}"
                    checkout scm

                    echo "Building Docker image: ${IMAGE_NAME}"
                    // Jenkins에 등록한 GitLab 레지스트리 인증 정보 사용
                    withCredentials([usernamePassword(credentialsId: 'gitlab-registry-credentials', usernameVariable: 'REGISTRY_USER', passwordVariable: 'REGISTRY_PASSWORD')]) {
                        sh "echo ${REGISTRY_PASSWORD} | docker login ${REGISTRY} -u ${REGISTRY_USER} --password-stdin"
                    }
                    
                    sh "docker build -t ${IMAGE_NAME} ."
                    sh "docker push ${IMAGE_NAME}"
                }
            }
        }
        
        stage('Deploy') {
            steps {
                script {
                    echo "Deploying image: ${IMAGE_NAME}"
                    // docker-compose가 사용할 이미지 이름을 환경변수로 주입
                    sh "export IMAGE_TO_DEPLOY=${IMAGE_NAME} && docker-compose down && docker-compose up --pull -d"
                }
            }
        }
    }

    post {
        always {
            echo "Cleaning up..."
            sh 'docker image prune -af'
        }
    }
}